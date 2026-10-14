#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "mesh_zigbee.h"
#include "i2c_manager.h"
#include "Compass.h"

static const char *TAG = "COMPASS";

// QMC5883L Registers
#define QMC5883L_REG_DATA_X_LSB  0x00
#define QMC5883L_REG_DATA_X_MSB  0x01
#define QMC5883L_REG_DATA_Y_LSB  0x02
#define QMC5883L_REG_DATA_Y_MSB  0x03
#define QMC5883L_REG_DATA_Z_LSB  0x04
#define QMC5883L_REG_DATA_Z_MSB  0x05
#define QMC5883L_REG_STATUS      0x06
#define QMC5883L_REG_CTRL1       0x09
#define QMC5883L_REG_CTRL2       0x0A
#define QMC5883L_REG_SET_RESET   0x0B
#define QMC5883L_REG_CHIP_ID     0x0D

static i2c_master_dev_handle_t s_dev_handle = NULL;
static uint8_t s_current_addr = QMC5883L_DEFAULT_I2C_ADDR;

static esp_err_t compass_write_reg(uint8_t reg, uint8_t val)
{
    if (s_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t tx_buf[2] = {reg, val};
    return i2c_master_transmit(s_dev_handle, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(100));
}

esp_err_t compass_init(uint8_t i2c_addr)
{
    uint8_t target_addr = (i2c_addr == 0) ? QMC5883L_DEFAULT_I2C_ADDR : i2c_addr;

    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus handle.");
        return ESP_FAIL;
    }

    if (s_dev_handle != NULL) {
        if (s_current_addr == target_addr) {
            ESP_LOGI(TAG, "Reusing existing QMC5883L handle at 0x%02X", target_addr);
        } else {
            i2c_master_bus_rm_device(s_dev_handle);
            s_dev_handle = NULL;
        }
    }

    if (s_dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = target_addr,
            .scl_speed_hz = 100000,
        };

        esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add QMC5883L I2C device: %s", esp_err_to_name(ret));
            return ret;
        }
        s_current_addr = target_addr;
    }

    // 1. Soft Reset: Write 0x80 to Control Register 2
    compass_write_reg(QMC5883L_REG_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(15));

    // 2. Set/Reset Period: Write 0x01 to Register 0x0B (per QMC5883L datasheet)
    esp_err_t ret = compass_write_reg(QMC5883L_REG_SET_RESET, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SET/RESET period: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Control Register 1: 0x1D
    // Bits 7:6 = 00 (OSR 512)
    // Bits 5:4 = 01 (RNG 8G)
    // Bits 3:2 = 11 (ODR 200Hz)
    // Bits 1:0 = 01 (Mode Continuous)
    ret = compass_write_reg(QMC5883L_REG_CTRL1, 0x1D);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Control Register 1: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. Control Register 2: Pointer roll-over enabled (0x40), interrupt disabled
    ret = compass_write_reg(QMC5883L_REG_CTRL2, 0x40);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Control Register 2: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "QMC5883L compass initialized successfully at 0x%02X (Continuous, 200Hz, 8G)", target_addr);
    return ESP_OK;
}

bool compass_is_initialized(void)
{
    return (s_dev_handle != NULL);
}

esp_err_t compass_read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    if (s_dev_handle == NULL) {
        esp_err_t err = compass_init(s_current_addr);
        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t reg = QMC5883L_REG_DATA_X_LSB;
    uint8_t data[6] = {0};

    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read magnetic field registers: %s", esp_err_to_name(ret));
        return ret;
    }

    // Data is 16-bit 2's complement, little-endian: [LSB, MSB]
    if (x) *x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    if (y) *y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    if (z) *z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    return ESP_OK;
}

bool compass_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "SetupCompass") == 0 ||
            strcasecmp(cmd, "GetCompass") == 0 ||
            strcasecmp(cmd, "ReadCompass") == 0);
}

bool compass_execute_cmd(const char *cmd, int16_t value, bool text_message)
{
    if (strcasecmp(cmd, "SetupCompass") == 0) {
        uint8_t addr = (value > 0) ? (uint8_t)value : QMC5883L_DEFAULT_I2C_ADDR;
        if (compass_init(addr) == ESP_OK) {
            mesh_zigbee_send_text("SETUP COMPASS OK");
        } else {
            mesh_zigbee_send_text("SETUP COMPASS FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "GetCompass") == 0 || strcasecmp(cmd, "ReadCompass") == 0) {
        int16_t x = 0, y = 0, z = 0;
        if (compass_read_mag(&x, &y, &z) == ESP_OK) {
            char reply[48];
            snprintf(reply, sizeof(reply), "COMPASS %d %d %d", x, y, z);
            mesh_zigbee_send_text(reply);
        } else {
            mesh_zigbee_send_text("COMPASS NOT READY");
        }
        return true;
    }
    return false;
}

