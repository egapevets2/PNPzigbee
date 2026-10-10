#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "apds9930.h"       // For get_shared_i2c_bus()
#include "UpstreamQ.h"      // For SendTheMessage()
#include "mesh_zigbee.h"    // For mesh_zigbee_send_text()
#include "Expand8.h"

static const char *TAG = "EXPAND8";

static i2c_master_dev_handle_t s_dev_handle = NULL;
static uint8_t s_current_addr = 0;
// PCF8574 default power-on state is 0xFF (quasi-bidirectional inputs / weak pull-ups)
static uint8_t s_port_state = 0xFF;

static uint8_t probe_pcf8574_address(i2c_master_bus_handle_t bus)
{
    // 1. Probe standard PCF8574 address range (0x20 .. 0x27)
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "Found PCF8574 at I2C address 0x%02X", addr);
            return addr;
        }
    }

    // 2. Probe PCF8574A address range (0x38 .. 0x3F)
    for (uint8_t addr = 0x38; addr <= 0x3F; addr++) {
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "Found PCF8574A at I2C address 0x%02X", addr);
            return addr;
        }
    }

    ESP_LOGW(TAG, "No PCF8574/A device acknowledged probe, defaulting to 0x%02X", PCF8574_DEFAULT_ADDR);
    return PCF8574_DEFAULT_ADDR;
}

esp_err_t expand8_init(uint8_t i2c_addr)
{
    i2c_master_bus_handle_t bus = get_shared_i2c_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus.");
        return ESP_FAIL;
    }

    uint8_t target_addr = i2c_addr;
    if (target_addr == 0 || target_addr < 0x08 || target_addr > 0x77) {
        target_addr = probe_pcf8574_address(bus);
    } else {
        esp_err_t probe_err = i2c_master_probe(bus, target_addr, pdMS_TO_TICKS(50));
        if (probe_err != ESP_OK) {
            ESP_LOGW(TAG, "Device probe at specified address 0x%02X returned %s",
                     target_addr, esp_err_to_name(probe_err));
        }
    }

    // Remove existing handle if already configured
    if (s_dev_handle != NULL) {
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = target_addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", target_addr, esp_err_to_name(ret));
        return ret;
    }

    s_current_addr = target_addr;
    s_port_state = 0xFF; // Set all bits high (inputs / idle)

    // Synchronize initial port state to the PCF8574
    ret = i2c_master_transmit(s_dev_handle, &s_port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write initial port state to 0x%02X: %s", target_addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PCF8574 initialized at address 0x%02X, port state 0x%02X", s_current_addr, s_port_state);
    return ESP_OK;
}

esp_err_t expand8_write_bit(uint8_t bit, uint8_t val)
{
    if (s_dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized. Please run SetupExpand8 first.");
        return ESP_ERR_INVALID_STATE;
    }
    if (bit > 7) {
        ESP_LOGE(TAG, "Invalid bit index %d (must be 0-7)", bit);
        return ESP_ERR_INVALID_ARG;
    }

    if (val) {
        s_port_state |= (1 << bit);
    } else {
        s_port_state &= ~(1 << bit);
    }

    esp_err_t ret = i2c_master_transmit(s_dev_handle, &s_port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write bit %d: %s", bit, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Expand8 bit %d set to %d (port=0x%02X)", bit, val ? 1 : 0, s_port_state);
    return ESP_OK;
}

esp_err_t expand8_read_bit(uint8_t bit, uint8_t *val)
{
    if (s_dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized. Please run SetupExpand8 first.");
        return ESP_ERR_INVALID_STATE;
    }
    if (bit > 7 || val == NULL) {
        ESP_LOGE(TAG, "Invalid argument (bit=%d)", bit);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port_val = 0;
    esp_err_t ret = i2c_master_receive(s_dev_handle, &port_val, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read port: %s", esp_err_to_name(ret));
        return ret;
    }

    *val = (port_val >> bit) & 0x01;
    ESP_LOGI(TAG, "Expand8 read bit %d = %d (port=0x%02X)", bit, *val, port_val);
    return ESP_OK;
}

esp_err_t expand8_write_port(uint8_t port_val)
{
    if (s_dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized. Please run SetupExpand8 first.");
        return ESP_ERR_INVALID_STATE;
    }

    s_port_state = port_val;
    esp_err_t ret = i2c_master_transmit(s_dev_handle, &s_port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write port: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Expand8 port set to 0x%02X", s_port_state);
    return ESP_OK;
}

esp_err_t expand8_read_port(uint8_t *port_val)
{
    if (s_dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized. Please run SetupExpand8 first.");
        return ESP_ERR_INVALID_STATE;
    }
    if (port_val == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_master_receive(s_dev_handle, port_val, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read port: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Expand8 port read 0x%02X", *port_val);
    return ESP_OK;
}

bool expand8_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "SetupExpand8") == 0 ||
            strcmp(cmd, "wrExpand8") == 0 ||
            strcmp(cmd, "rdExpand8") == 0 ||
            strcmp(cmd, "setExpand8") == 0 ||
            strcmp(cmd, "clrExpand8") == 0 ||
            strcmp(cmd, "wrExpand8byte") == 0 ||
            strcmp(cmd, "rdExpand8byte") == 0);
}

bool expand8_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcmp(cmd, "SetupExpand8") == 0) {
        esp_err_t ret = expand8_init((uint8_t)value);
        if (ret == ESP_OK) {
            mesh_zigbee_send_text("SETUP EXPAND8 OK");
        } else {
            mesh_zigbee_send_text("SETUP EXPAND8 FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "wrExpand8") == 0) {
        if (s_dev_handle == NULL) {
            if (expand8_init(0) != ESP_OK) {
                if (!text_message) mesh_zigbee_send_text("EXPAND8 NOT READY");
                return true;
            }
        }
        if (value < 0 || value > 7) {
            ESP_LOGE(TAG, "wrExpand8 bit %d out of range [0..7]", value);
            if (!text_message) mesh_zigbee_send_text("ERR: INVALID BIT");
            return true;
        }

        // If value2 was 0, write 0. If value2 > 0 or omitted (-1), write 1.
        uint8_t bit_val = (value2 <= 0 && value2 != -1) ? 0 : 1;
        if (expand8_write_bit((uint8_t)value, bit_val) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "rdExpand8") == 0) {
        if (s_dev_handle == NULL) {
            if (expand8_init(0) != ESP_OK) {
                SendTheMessage("EXPAND8 NOT READY");
                return true;
            }
        }
        if (value < 0 || value > 7) {
            ESP_LOGE(TAG, "rdExpand8 bit %d out of range [0..7]", value);
            SendTheMessage("ERR: INVALID BIT");
            return true;
        }

        uint8_t bit_val = 0;
        if (expand8_read_bit((uint8_t)value, &bit_val) == ESP_OK) {
            char reply[32];
            snprintf(reply, sizeof(reply), "EXPAND8 %d IS %d", value, bit_val);
            SendTheMessage(reply);
        } else {
            SendTheMessage("EXPAND8 RD FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "setExpand8") == 0) {
        if (s_dev_handle == NULL) expand8_init(0);
        if (value >= 0 && value <= 7) {
            expand8_write_bit((uint8_t)value, 1);
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        }
        return true;
    }
    else if (strcmp(cmd, "clrExpand8") == 0) {
        if (s_dev_handle == NULL) expand8_init(0);
        if (value >= 0 && value <= 7) {
            expand8_write_bit((uint8_t)value, 0);
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        }
        return true;
    }
    else if (strcmp(cmd, "wrExpand8byte") == 0) {
        if (s_dev_handle == NULL) expand8_init(0);
        if (expand8_write_port((uint8_t)value) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "rdExpand8byte") == 0) {
        if (s_dev_handle == NULL) expand8_init(0);
        uint8_t port_val = 0;
        if (expand8_read_port(&port_val) == ESP_OK) {
            char reply[32];
            snprintf(reply, sizeof(reply), "EXPAND8 BYTE 0x%02X", port_val);
            SendTheMessage(reply);
        } else {
            SendTheMessage("EXPAND8 RD FAIL");
        }
        return true;
    }

    return false;
}

