/*
I2Craw.c
This gives access to the coordinator to put writes onto the i2c bus,
by sending commnands.

sets the chip address for the device you are going to write or read
rawI2CchipADR 21

rawI2Cwr  17 197
writes I2C value 197 to register address 17 of the device with I2C addr "rawI2CchipADR"

rawI2Crd 2 
reads the value in register address 2 of the device with I2C addr "rawI2CchipADR"
*/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mesh_zigbee.h"
#include "apds9930.h" // For get_shared_i2c_bus()
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

    i2c_master_bus_handle_t bus = get_shared_i2c_bus();
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
            strcmp(cmd, "rawI2Crd") == 0);
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
        if (i2c_master_transmit(current_dev_handle, data, 2, -1) == ESP_OK) {
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
        if (i2c_master_transmit_receive(current_dev_handle, &reg, 1, &data, 1, -1) == ESP_OK) {
            snprintf(reply, sizeof(reply), "I2Cval %d %d", reg, data);
            mesh_zigbee_send_text(reply);
        } else {
            mesh_zigbee_send_text("I2C RD FAIL");
        }
        return true;
    }
    return false;
}