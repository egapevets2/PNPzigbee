/*
 * I2Craw.c - Direct Low-Level I2C Register Access & Bus Scanner
 *
 * Purpose:
 *   Provides low-level access to the shared I2C bus for the coordinator or automated test harness.
 *   Allows dynamically binding to any 7-bit slave address, writing arbitrary register/data bytes,
 *   reading registers, and scanning the entire 7-bit address range for responding devices.
 *
 * Supported Commands:
 *   rawI2CchipADR <addr>
 *     - addr: 7-bit I2C target slave address (e.g. 21 or 0x15).
 *     Sets the active target I2C address for subsequent raw read and write operations.
 *     Replies "I2C CHIP ADR OK" or "I2C CHIP ADR FAIL".
 *
 *   rawI2Cwr <reg> <val>
 *     - reg: Target device 8-bit register address.
 *     - val: 8-bit value to write.
 *     Transmits 2-byte sequence [reg, val] to the current target chip address.
 *     Replies "I2C WR OK" or "I2C WR FAIL".
 *
 *   rawI2Crd <reg>
 *     - reg: Target device 8-bit register address.
 *     Writes register address and reads back 1 data byte from the target chip address.
 *     Replies "I2Cval <reg> <val>".
 *
 *   scanI2C / I2Cscan
 *     Probes all 7-bit I2C addresses from 0x08 to 0x77 with a 20ms timeout.
 *     Replies with all responding addresses, e.g. "I2C SCAN: 0x0D(13) 0x3C(60)" or "I2C SCAN: NONE".
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "mesh_zigbee.h"
#include "i2c_manager.h"
#include "I2Craw.h"

static const char *TAG = "I2CRAW";

static uint8_t current_chip_addr = 0;
static i2c_master_dev_handle_t current_dev_handle = NULL;

// Helper to dynamically attach to a new target I2C address
static esp_err_t ensure_device_handle(uint8_t addr)
{
    // If we are already attached to this address, do nothing
    if (current_dev_handle != NULL && current_chip_addr == addr) {
        return ESP_OK;
    }

    // If we are changing addresses, clean up the old device handle
    if (current_dev_handle != NULL) {
        i2c_master_bus_rm_device(current_dev_handle);
        current_dev_handle = NULL;
    }

    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus.");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &current_dev_handle);
    if (ret == ESP_OK) {
        current_chip_addr = addr;
        ESP_LOGI(TAG, "I2C target address set to 0x%02X", addr);
    } else {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X", addr);
    }
    return ret;
}

bool I2Craw_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "rawI2CchipADR") == 0 ||
            strcmp(cmd, "rawI2Cwr") == 0 ||
            strcmp(cmd, "rawI2Crd") == 0 ||
            strcasecmp(cmd, "scanI2C") == 0 ||
            strcasecmp(cmd, "I2Cscan") == 0);
}

bool I2Craw_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    char reply[32];

    if (strcmp(cmd, "rawI2CchipADR") == 0) {
        if (ensure_device_handle((uint8_t)value) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("I2C CHIP ADR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("I2C CHIP ADR FAIL");
        }
        return true;
    } 
    else if (strcmp(cmd, "rawI2Cwr") == 0) {
        if (current_dev_handle == NULL) {
            if (!text_message) mesh_zigbee_send_text("ERR: NO I2C ADR SET");
            return true;
        }
        uint8_t data[2] = {(uint8_t)value, (uint8_t)value2};
        if (i2c_master_transmit(current_dev_handle, data, 2, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (!text_message) mesh_zigbee_send_text("I2C WR OK");
        } else {
            if (!text_message) mesh_zigbee_send_text("I2C WR FAIL");
        }
        return true;
    } 
    else if (strcmp(cmd, "rawI2Crd") == 0) {
        if (current_dev_handle == NULL) {
            mesh_zigbee_send_text("ERR: NO I2C ADR SET");
            return true;
        }
        uint8_t reg = (uint8_t)value;
        uint8_t data = 0;
        if (i2c_master_transmit_receive(current_dev_handle, &reg, 1, &data, 1, pdMS_TO_TICKS(100)) == ESP_OK) {
            snprintf(reply, sizeof(reply), "I2Cval %d %d", reg, data);
            mesh_zigbee_send_text(reply);
        } else {
            mesh_zigbee_send_text("I2C RD FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "scanI2C") == 0 || strcasecmp(cmd, "I2Cscan") == 0) {
        if (i2c_manager_init_bus() != ESP_OK) {
            mesh_zigbee_send_text("I2C BUS INIT FAIL");
            return true;
        }
        i2c_master_bus_handle_t bus = i2c_manager_get_bus();
        if (bus == NULL) {
            mesh_zigbee_send_text("I2C BUS GET FAIL");
            return true;
        }

        char found_buf[128] = "I2C SCAN:";
        int found_count = 0;

        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            // Note: timeout parameter in i2c_master_probe is in milliseconds
            esp_err_t probe_ret = i2c_master_probe(bus, addr, 20);
            if (probe_ret == ESP_OK) {
                found_count++;
                char entry[24];
                snprintf(entry, sizeof(entry), " 0x%02X(%d)", addr, addr);
                if (strlen(found_buf) + strlen(entry) < sizeof(found_buf) - 1) {
                    strcat(found_buf, entry);
                }
                ESP_LOGI(TAG, "I2C device found at address 0x%02X (%d)", addr, addr);
            }
        }

        if (found_count == 0) {
            mesh_zigbee_send_text("I2C SCAN: NONE");
            ESP_LOGI(TAG, "I2C scan complete: no devices found.");
        } else {
            mesh_zigbee_send_text(found_buf);
            ESP_LOGI(TAG, "I2C scan complete: %s", found_buf);
        }
        return true;
    }
    return false;
}