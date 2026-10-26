/*
 * Expand8.c - PCF8574 / PCF8574A 8-Bit I2C GPIO Expander Driver
 *
 * Purpose:
 *   Drives Texas Instruments / NXP PCF8574 (0x20..0x27) and PCF8574A (0x38..0x3F)
 *   remote 8-bit I/O expanders over the shared I2C bus. Supports multiple concurrent expander
 *   instances, automatic address probing on the bus, individual bit read/write/set/clear,
 *   and full 8-bit byte read/write operations.
 *
 * Supported Commands:
 *   SetupExpand8 [addr]
 *     - addr: 7-bit I2C address (0x20..0x27 or 0x38..0x3F). If 0 or omitted, auto-probes the bus.
 *     Initializes PCF8574/A hardware device and caches instance handle.
 *     Replies "SETUP EXPAND8 OK" or "SETUP EXPAND8 FAIL".
 *
 *   wrExpand8 <bit> <val> [addr]  /  wrExpand8 <addr> <bit> [val]
 *     - bit: Pin index (0..7).
 *     - val: Logic level (0 or 1).
 *     - addr: (Optional) Target expander I2C address if multiple chips present.
 *     Writes logic state to the specified expander pin.
 *
 *   rdExpand8 <bit> [addr]  /  rdExpand8 <addr> <bit>
 *     - bit: Pin index (0..7).
 *     - addr: (Optional) Target expander I2C address.
 *     Reads logic state of the specified expander pin.
 *     Replies "EXPAND8 <bit> IS <val>" (0 or 1).
 *
 *   setExpand8 <bit> [addr]
 *     - bit: Pin index (0..7) to drive HIGH (1).
 *
 *   clrExpand8 <bit> [addr]
 *     - bit: Pin index (0..7) to drive LOW (0).
 *
 *   wrExpand8byte <val> [addr]
 *     - val: 8-bit port byte value (0..255 / 0x00..0xFF).
 *     - addr: (Optional) Target expander I2C address.
 *     Writes all 8 pins simultaneously.
 *
 *   rdExpand8byte [addr]
 *     - addr: (Optional) Target expander I2C address.
 *     Reads full 8-bit port state.
 *     Replies "EXPAND8 BYTE 0x<HEX>".
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_manager.h"
#include "apds9930.h"       // For APDS9930_I2C_ADDR
#include "VL53L1X.h"        // For VL53L1X_I2C_ADDR
#include "UpstreamQ.h"      // For SendTheMessage()
#include "mesh_zigbee.h"    // For mesh_zigbee_send_text()
#include "Expand8.h"

static const char *TAG = "EXPAND8";

typedef struct {
    bool initialized;
    uint8_t i2c_addr;
    uint8_t port_state; // PCF8574 default power-on state is 0xFF (quasi-bidirectional inputs / weak pull-ups)
    i2c_master_dev_handle_t dev_handle;
} expand8_instance_t;

static expand8_instance_t s_instances[EXPAND8_MAX_INSTANCES] = {0};
static int s_active_idx = -1;

bool expand8_check_latched_bus(esp_err_t err)
{
    return i2c_manager_check_latched_bus("EXPAND8", err);
}

static uint8_t probe_pcf8574_address(i2c_master_bus_handle_t bus)
{
    // 1. Probe standard PCF8574 address range (0x20 .. 0x27)
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "Found PCF8574 at I2C address 0x%02X", addr);
            return addr;
        }
    }

    // 2. Probe PCF8574A address range (0x38 .. 0x3F), explicitly skipping APDS9930 (0x39)
    for (uint8_t addr = 0x38; addr <= 0x3F; addr++) {
        if (addr == APDS9930_I2C_ADDR) {
            continue; // Skip fixed APDS-9930 optical sensor address
        }
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(20)) == ESP_OK) {
            ESP_LOGI(TAG, "Found PCF8574A at I2C address 0x%02X", addr);
            return addr;
        }
    }

    ESP_LOGW(TAG, "No PCF8574/A device acknowledged probe, defaulting to 0x%02X", PCF8574_DEFAULT_ADDR);
    return PCF8574_DEFAULT_ADDR;
}

static int find_instance_by_addr(uint8_t addr)
{
    for (int i = 0; i < EXPAND8_MAX_INSTANCES; i++) {
        if (s_instances[i].initialized && s_instances[i].i2c_addr == addr) {
            return i;
        }
    }
    return -1;
}

esp_err_t expand8_init(uint8_t i2c_addr)
{
    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus.");
        return ESP_FAIL;
    }

    uint8_t target_addr = i2c_addr;
    if (target_addr == 0) {
        // Explicit default address (0x20 = 32 decimal) prevents bus clash during unparameterized setup
        target_addr = PCF8574_DEFAULT_ADDR;
    } else if (target_addr == 0xFF || target_addr < 0x08 || target_addr > 0x77) {
        target_addr = probe_pcf8574_address(bus);
    }

    esp_err_t probe_err = i2c_master_probe(bus, target_addr, pdMS_TO_TICKS(50));
    if (probe_err != ESP_OK) {
        ESP_LOGW(TAG, "Device probe at address 0x%02X (dec %d) returned %s",
                 target_addr, target_addr, esp_err_to_name(probe_err));
    }

    // Select slot: reuse existing slot for target_addr, or find a free slot, or replace non-active slot
    int slot = find_instance_by_addr(target_addr);
    if (slot < 0) {
        for (int i = 0; i < EXPAND8_MAX_INSTANCES; i++) {
            if (!s_instances[i].initialized) {
                slot = i;
                break;
            }
        }
    }
    if (slot < 0) {
        // All slots occupied: replace the slot that is not currently active
        slot = (s_active_idx == 0 && EXPAND8_MAX_INSTANCES > 1) ? 1 : 0;
        ESP_LOGI(TAG, "Reclaiming Expand8 instance slot %d (was 0x%02X) for new address 0x%02X",
                 slot, s_instances[slot].i2c_addr, target_addr);
    }

    expand8_instance_t *inst = &s_instances[slot];
    if (inst->dev_handle != NULL) {
        i2c_master_bus_rm_device(inst->dev_handle);
        inst->dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = target_addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &inst->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", target_addr, esp_err_to_name(ret));
        inst->initialized = false;
        return ret;
    }

    inst->i2c_addr = target_addr;
    inst->port_state = 0xFF; // Set all bits high (inputs / idle)
    inst->initialized = true;
    s_active_idx = slot;

    // Synchronize initial port state to the PCF8574
    ret = i2c_master_transmit(inst->dev_handle, &inst->port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write initial port state to 0x%02X: %s", target_addr, esp_err_to_name(ret));
        expand8_check_latched_bus(ret);
        return ret;
    }
    expand8_check_latched_bus(ESP_OK);

    ESP_LOGI(TAG, "PCF8574 instance[%d] initialized at address 0x%02X (%d), port state 0x%02X",
             slot, inst->i2c_addr, inst->i2c_addr, inst->port_state);
    return ESP_OK;
}

static expand8_instance_t *resolve_instance(uint8_t i2c_addr)
{
    // Enforce Zero-I2C boot isolation: if SetupExpand8 has never been called, reject without touching GPIO 22/23
    if (s_active_idx < 0) {
        return NULL;
    }

    if (i2c_addr == 0) {
        if (s_active_idx >= 0 && s_active_idx < EXPAND8_MAX_INSTANCES && s_instances[s_active_idx].initialized) {
            return &s_instances[s_active_idx];
        }
        return NULL;
    }

    int idx = find_instance_by_addr(i2c_addr);
    if (idx >= 0) {
        s_active_idx = idx;
        return &s_instances[idx];
    }

    // If Expand8 subsystem is already active and a valid second chip address is passed on the fly, attach it
    if (i2c_addr >= 0x08 && i2c_addr <= 0x77) {
        if (expand8_init(i2c_addr) == ESP_OK && s_active_idx >= 0) {
            return &s_instances[s_active_idx];
        }
    }

    return NULL;
}

esp_err_t expand8_write_bit_addr(uint8_t i2c_addr, uint8_t bit, uint8_t val)
{
    expand8_instance_t *inst = resolve_instance(i2c_addr);
    if (inst == NULL || inst->dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized for addr 0x%02X. Please run SetupExpand8 first.", i2c_addr);
        return ESP_ERR_INVALID_STATE;
    }
    if (bit > 7) {
        ESP_LOGE(TAG, "Invalid bit index %d (must be 0-7)", bit);
        return ESP_ERR_INVALID_ARG;
    }

    if (val) {
        inst->port_state |= (1 << bit);
    } else {
        inst->port_state &= ~(1 << bit);
    }

    esp_err_t ret = i2c_master_transmit(inst->dev_handle, &inst->port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write bit %d to 0x%02X: %s", bit, inst->i2c_addr, esp_err_to_name(ret));
        expand8_check_latched_bus(ret);
        return ret;
    }
    expand8_check_latched_bus(ESP_OK);

    ESP_LOGI(TAG, "Expand8[0x%02X] bit %d set to %d (port=0x%02X)", inst->i2c_addr, bit, val ? 1 : 0, inst->port_state);
    return ESP_OK;
}

esp_err_t expand8_write_bit(uint8_t bit, uint8_t val)
{
    return expand8_write_bit_addr(0, bit, val);
}

esp_err_t expand8_read_bit_addr(uint8_t i2c_addr, uint8_t bit, uint8_t *val)
{
    expand8_instance_t *inst = resolve_instance(i2c_addr);
    if (inst == NULL || inst->dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized for addr 0x%02X. Please run SetupExpand8 first.", i2c_addr);
        return ESP_ERR_INVALID_STATE;
    }
    if (bit > 7 || val == NULL) {
        ESP_LOGE(TAG, "Invalid argument (bit=%d)", bit);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t port_val = 0;
    esp_err_t ret = i2c_master_receive(inst->dev_handle, &port_val, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read port from 0x%02X: %s", inst->i2c_addr, esp_err_to_name(ret));
        expand8_check_latched_bus(ret);
        return ret;
    }
    expand8_check_latched_bus(ESP_OK);

    *val = (port_val >> bit) & 0x01;
    ESP_LOGI(TAG, "Expand8[0x%02X] read bit %d = %d (port=0x%02X)", inst->i2c_addr, bit, *val, port_val);
    return ESP_OK;
}

esp_err_t expand8_read_bit(uint8_t bit, uint8_t *val)
{
    return expand8_read_bit_addr(0, bit, val);
}

esp_err_t expand8_write_port_addr(uint8_t i2c_addr, uint8_t port_val)
{
    expand8_instance_t *inst = resolve_instance(i2c_addr);
    if (inst == NULL || inst->dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized for addr 0x%02X. Please run SetupExpand8 first.", i2c_addr);
        return ESP_ERR_INVALID_STATE;
    }

    inst->port_state = port_val;
    esp_err_t ret = i2c_master_transmit(inst->dev_handle, &inst->port_state, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write port to 0x%02X: %s", inst->i2c_addr, esp_err_to_name(ret));
        expand8_check_latched_bus(ret);
        return ret;
    }
    expand8_check_latched_bus(ESP_OK);

    ESP_LOGI(TAG, "Expand8[0x%02X] port set to 0x%02X", inst->i2c_addr, inst->port_state);
    return ESP_OK;
}

esp_err_t expand8_write_port(uint8_t port_val)
{
    return expand8_write_port_addr(0, port_val);
}

esp_err_t expand8_read_port_addr(uint8_t i2c_addr, uint8_t *port_val)
{
    expand8_instance_t *inst = resolve_instance(i2c_addr);
    if (inst == NULL || inst->dev_handle == NULL) {
        ESP_LOGE(TAG, "Expand8 not initialized for addr 0x%02X. Please run SetupExpand8 first.", i2c_addr);
        return ESP_ERR_INVALID_STATE;
    }
    if (port_val == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = i2c_master_receive(inst->dev_handle, port_val, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read port from 0x%02X: %s", inst->i2c_addr, esp_err_to_name(ret));
        expand8_check_latched_bus(ret);
        return ret;
    }
    expand8_check_latched_bus(ESP_OK);

    ESP_LOGI(TAG, "Expand8[0x%02X] port read 0x%02X", inst->i2c_addr, *port_val);
    return ESP_OK;
}

esp_err_t expand8_read_port(uint8_t *port_val)
{
    return expand8_read_port_addr(0, port_val);
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

bool expand8_execute_cmd(const char *cmd, int16_t value, int16_t value2, int16_t value3, bool text_message)
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
        if (s_active_idx < 0) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 NOT READY");
            return true;
        }

        // Supports both:
        //   wrExpand8 <bit> <val> [addr]  (e.g. wrExpand8 3 1 33)
        //   wrExpand8 <addr> <bit> [val]  (e.g. wrExpand8 33 3 1)
        int16_t bit = value;
        int16_t val_arg = value2;
        uint8_t addr = 0;

        if (value >= 8 && value <= 0x77 && value2 >= 0 && value2 <= 7) {
            addr = (uint8_t)value;
            bit = value2;
            val_arg = value3;
        } else if (value3 >= 8 && value3 <= 0x77) {
            addr = (uint8_t)value3;
        }

        if (bit < 0 || bit > 7) {
            ESP_LOGE(TAG, "wrExpand8 bit %d out of range [0..7]", bit);
            if (!text_message) mesh_zigbee_send_text("ERR: INVALID BIT");
            return true;
        }

        // If val_arg was 0, write 0. If val_arg > 0 or omitted (-1), write 1.
        uint8_t bit_val = (val_arg <= 0 && val_arg != -1) ? 0 : 1;
        if (expand8_write_bit_addr(addr, (uint8_t)bit, bit_val) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "rdExpand8") == 0) {
        if (s_active_idx < 0) {
            SendTheMessage("EXPAND8 NOT READY");
            return true;
        }

        // Supports both:
        //   rdExpand8 <bit> [addr]  (e.g. rdExpand8 3 33)
        //   rdExpand8 <addr> <bit>  (e.g. rdExpand8 33 3)
        int16_t bit = value;
        uint8_t addr = 0;

        if (value >= 8 && value <= 0x77 && value2 >= 0 && value2 <= 7) {
            addr = (uint8_t)value;
            bit = value2;
        } else if (value2 >= 8 && value2 <= 0x77) {
            addr = (uint8_t)value2;
        }

        if (bit < 0 || bit > 7) {
            ESP_LOGE(TAG, "rdExpand8 bit %d out of range [0..7]", bit);
            SendTheMessage("ERR: INVALID BIT");
            return true;
        }

        uint8_t bit_val = 0;
        if (expand8_read_bit_addr(addr, (uint8_t)bit, &bit_val) == ESP_OK) {
            char reply[32];
            snprintf(reply, sizeof(reply), "EXPAND8 %d IS %d", bit, bit_val);
            SendTheMessage(reply);
        } else {
            SendTheMessage("EXPAND8 RD FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "setExpand8") == 0) {
        if (s_active_idx < 0) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 NOT READY");
            return true;
        }

        int16_t bit = value;
        uint8_t addr = 0;
        if (value >= 8 && value <= 0x77 && value2 >= 0 && value2 <= 7) {
            addr = (uint8_t)value;
            bit = value2;
        } else if (value2 >= 8 && value2 <= 0x77) {
            addr = (uint8_t)value2;
        }

        if (bit >= 0 && bit <= 7) {
            expand8_write_bit_addr(addr, (uint8_t)bit, 1);
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        }
        return true;
    }
    else if (strcmp(cmd, "clrExpand8") == 0) {
        if (s_active_idx < 0) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 NOT READY");
            return true;
        }

        int16_t bit = value;
        uint8_t addr = 0;
        if (value >= 8 && value <= 0x77 && value2 >= 0 && value2 <= 7) {
            addr = (uint8_t)value;
            bit = value2;
        } else if (value2 >= 8 && value2 <= 0x77) {
            addr = (uint8_t)value2;
        }

        if (bit >= 0 && bit <= 7) {
            expand8_write_bit_addr(addr, (uint8_t)bit, 0);
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        }
        return true;
    }
    else if (strcmp(cmd, "wrExpand8byte") == 0) {
        if (s_active_idx < 0) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 NOT READY");
            return true;
        }
        uint8_t addr = (value2 >= 8 && value2 <= 0x77) ? (uint8_t)value2 : 0;
        if (expand8_write_port_addr(addr, (uint8_t)value) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("EXPAND8 WR FAIL");
        }
        return true;
    }
    else if (strcmp(cmd, "rdExpand8byte") == 0) {
        if (s_active_idx < 0) {
            SendTheMessage("EXPAND8 NOT READY");
            return true;
        }
        uint8_t addr = (value >= 8 && value <= 0x77) ? (uint8_t)value : 0;
        uint8_t port_val = 0;
        if (expand8_read_port_addr(addr, &port_val) == ESP_OK) {
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

