/*
 * I2Cmux.c - TCA9548A 8-Channel I2C Multiplexer / Switch Driver
 *
 * Purpose:
 *   Drives Texas Instruments TCA9548A (0x70..0x77) 8-channel I2C switches
 *   over the shared I2C bus (i2c_manager). Allows dynamically choosing one
 *   of the 8 downstream I2C channels (0..7) to communicate with conflicting
 *   or multiplexed I2C target devices.
 *
 * Supported Commands:
 *   SetI2Cmux <X> [addr]  (or SetI2Cmux <addr> <X>)
 *     - X: Channel index [0..7] (or 1..8 for 1-based index).
 *          Pass -1 or 255 to disable all channels.
 *     - addr: (Optional) 7-bit I2C address (0x70..0x77, default: 0x70).
 *     Selects the specified I2C channel.
 *     Replies "SET I2CMUX OK" or "SET I2CMUX FAIL".
 *
 *   GetI2Cmux [addr] / ReadI2Cmux [addr]
 *     - addr: (Optional) 7-bit I2C address.
 *     Reads the current active channel from the TCA9548A control register.
 *     Replies "I2CMUX CH <X>", "I2CMUX NONE", or "I2CMUX MASK 0x<HEX>".
 *
 *   SetupI2Cmux [addr]
 *     - addr: (Optional) 7-bit I2C address (default: 0x70, or auto-probe).
 *     Initializes I2C bus and verifies TCA9548A presence.
 *     Replies "SETUP I2CMUX OK" or "SETUP I2CMUX FAIL".
 *
 *   SetI2CmuxMask <mask> [addr]
 *     - mask: Raw 8-bit channel mask (0x00..0xFF).
 *     Replies "SET I2CMUX OK" or "SET I2CMUX FAIL".
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_manager.h"
#include "mesh_zigbee.h"
#include "I2Cmux.h"

static const char *TAG = "I2CMUX";

static uint8_t s_current_addr = 0;
static i2c_master_dev_handle_t s_mux_dev_handle = NULL;
static uint8_t s_current_mask = 0;
static bool s_initialized = false;

static uint8_t probe_tca9548a_address(i2c_master_bus_handle_t bus)
{
    for (uint8_t addr = TCA9548A_MIN_ADDR; addr <= TCA9548A_MAX_ADDR; addr++) {
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "Found TCA9548A at I2C address 0x%02X", addr);
            return addr;
        }
    }
    ESP_LOGW(TAG, "No TCA9548A acknowledged probe (0x70..0x77), defaulting to 0x%02X", TCA9548A_DEFAULT_ADDR);
    return TCA9548A_DEFAULT_ADDR;
}

static esp_err_t ensure_mux_handle(uint8_t target_addr)
{
    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus handle.");
        return ESP_FAIL;
    }

    if (target_addr == 0) {
        if (s_initialized && s_current_addr >= TCA9548A_MIN_ADDR && s_current_addr <= TCA9548A_MAX_ADDR) {
            target_addr = s_current_addr;
        } else {
            target_addr = probe_tca9548a_address(bus);
        }
    }

    if (s_mux_dev_handle != NULL && s_current_addr == target_addr) {
        return ESP_OK; // Already configured
    }

    if (s_mux_dev_handle != NULL) {
        i2c_master_bus_rm_device(s_mux_dev_handle);
        s_mux_dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = target_addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_mux_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add TCA9548A I2C device at 0x%02X: %s", target_addr, esp_err_to_name(ret));
        s_mux_dev_handle = NULL;
        return ret;
    }

    s_current_addr = target_addr;
    ESP_LOGI(TAG, "TCA9548A device handle created at address 0x%02X", target_addr);
    return ESP_OK;
}

esp_err_t i2cmux_init(uint8_t i2c_addr)
{
    esp_err_t ret = ensure_mux_handle(i2c_addr);
    if (ret != ESP_OK) {
        return ret;
    }

    // Verify presence via probe
    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    ret = i2c_master_probe(bus, s_current_addr, pdMS_TO_TICKS(50));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TCA9548A probe at 0x%02X returned %s", s_current_addr, esp_err_to_name(ret));
    }

    // Initialize with all channels disabled (0x00)
    uint8_t mask = 0x00;
    ret = i2c_master_transmit(s_mux_dev_handle, &mask, 1, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        s_current_mask = 0x00;
        s_initialized = true;
        ESP_LOGI(TAG, "TCA9548A at 0x%02X initialized (all channels off)", s_current_addr);
    } else {
        ESP_LOGE(TAG, "Failed to write initial control byte to TCA9548A at 0x%02X: %s",
                 s_current_addr, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2cmux_set_mask_addr(uint8_t i2c_addr, uint8_t mask)
{
    esp_err_t ret = ensure_mux_handle(i2c_addr);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_transmit(s_mux_dev_handle, &mask, 1, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        s_current_mask = mask;
        s_initialized = true;
        ESP_LOGI(TAG, "TCA9548A (0x%02X) set mask = 0x%02X", s_current_addr, mask);
    } else {
        ESP_LOGE(TAG, "TCA9548A (0x%02X) transmit fail (mask 0x%02X): %s",
                 s_current_addr, mask, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2cmux_select_channel_addr(uint8_t i2c_addr, int8_t channel)
{
    uint8_t mask = 0x00;

    if (channel == -1 || channel == (int8_t)255) {
        // Disable all channels
        mask = 0x00;
    } else if (channel == 8) {
        // Map 1-based index 8 to channel 7
        mask = (1 << 7);
    } else if (channel >= 0 && channel < TCA9548A_NUM_CHANNELS) {
        // Standard 0-based channel 0..7
        mask = (1 << channel);
    } else {
        ESP_LOGE(TAG, "Invalid channel index %d (valid: 0..7, or -1 to disable)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    return i2cmux_set_mask_addr(i2c_addr, mask);
}

esp_err_t i2cmux_select_channel(int8_t channel)
{
    return i2cmux_select_channel_addr(0, channel);
}

esp_err_t i2cmux_get_mask_addr(uint8_t i2c_addr, uint8_t *mask)
{
    if (!mask) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ensure_mux_handle(i2c_addr);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_receive(s_mux_dev_handle, mask, 1, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        s_current_mask = *mask;
        s_initialized = true;
        ESP_LOGI(TAG, "TCA9548A (0x%02X) read mask = 0x%02X", s_current_addr, *mask);
    } else {
        ESP_LOGE(TAG, "TCA9548A (0x%02X) receive fail: %s", s_current_addr, esp_err_to_name(ret));
    }
    return ret;
}

int8_t i2cmux_get_selected_channel(void)
{
    if (s_current_mask == 0x00) {
        return -1; // All channels disabled
    }

    // Check if exactly one channel bit is set
    int bit_count = 0;
    int8_t active_ch = -1;
    for (int i = 0; i < TCA9548A_NUM_CHANNELS; i++) {
        if (s_current_mask & (1 << i)) {
            bit_count++;
            active_ch = i;
        }
    }

    return (bit_count == 1) ? active_ch : -1;
}

bool i2cmux_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "SetI2Cmux") == 0 ||
            strcasecmp(cmd, "GetI2Cmux") == 0 ||
            strcasecmp(cmd, "ReadI2Cmux") == 0 ||
            strcasecmp(cmd, "SetupI2Cmux") == 0 ||
            strcasecmp(cmd, "SetI2CmuxMask") == 0);
}

bool i2cmux_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (!cmd) return false;

    if (strcasecmp(cmd, "SetI2Cmux") == 0) {
        int8_t channel = (int8_t)value;
        uint8_t addr = 0;

        // Supports both:
        //   SetI2Cmux <channel> [addr]   (e.g. SetI2Cmux 2 0x70)
        //   SetI2Cmux <addr> <channel>   (e.g. SetI2Cmux 0x70 2)
        if (value >= TCA9548A_MIN_ADDR && value <= TCA9548A_MAX_ADDR &&
            value2 >= -1 && value2 <= TCA9548A_NUM_CHANNELS) {
            addr = (uint8_t)value;
            channel = (int8_t)value2;
        } else if (value2 >= TCA9548A_MIN_ADDR && value2 <= TCA9548A_MAX_ADDR) {
            addr = (uint8_t)value2;
            channel = (int8_t)value;
        }

        esp_err_t ret = i2cmux_select_channel_addr(addr, channel);
        if (ret == ESP_OK) {
            mesh_zigbee_send_text("SET I2CMUX OK");
        } else {
            mesh_zigbee_send_text("SET I2CMUX FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "GetI2Cmux") == 0 || strcasecmp(cmd, "ReadI2Cmux") == 0) {
        uint8_t addr = 0;
        if (value >= TCA9548A_MIN_ADDR && value <= TCA9548A_MAX_ADDR) {
            addr = (uint8_t)value;
        }

        uint8_t mask = 0;
        esp_err_t ret = i2cmux_get_mask_addr(addr, &mask);
        if (ret == ESP_OK) {
            char reply[32];
            int bit_count = 0;
            int single_ch = -1;
            for (int i = 0; i < TCA9548A_NUM_CHANNELS; i++) {
                if (mask & (1 << i)) {
                    bit_count++;
                    single_ch = i;
                }
            }

            if (bit_count == 1) {
                snprintf(reply, sizeof(reply), "I2CMUX CH %d", single_ch);
            } else if (mask == 0) {
                snprintf(reply, sizeof(reply), "I2CMUX NONE");
            } else {
                snprintf(reply, sizeof(reply), "I2CMUX MASK 0x%02X", mask);
            }
            mesh_zigbee_send_text(reply);
        } else {
            mesh_zigbee_send_text("I2CMUX RD FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "SetupI2Cmux") == 0) {
        uint8_t addr = 0;
        if (value >= TCA9548A_MIN_ADDR && value <= TCA9548A_MAX_ADDR) {
            addr = (uint8_t)value;
        }

        esp_err_t ret = i2cmux_init(addr);
        if (ret == ESP_OK) {
            mesh_zigbee_send_text("SETUP I2CMUX OK");
        } else {
            mesh_zigbee_send_text("SETUP I2CMUX FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "SetI2CmuxMask") == 0) {
        uint8_t mask = (uint8_t)value;
        uint8_t addr = 0;
        if (value2 >= TCA9548A_MIN_ADDR && value2 <= TCA9548A_MAX_ADDR) {
            addr = (uint8_t)value2;
        }

        esp_err_t ret = i2cmux_set_mask_addr(addr, mask);
        if (ret == ESP_OK) {
            mesh_zigbee_send_text("SET I2CMUX OK");
        } else {
            mesh_zigbee_send_text("SET I2CMUX FAIL");
        }
        return true;
    }

    return false;
}

