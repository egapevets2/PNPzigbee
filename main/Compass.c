/*
 * Compass.c - QMC5883L 3-Axis Digital Magnetometer Subsystem
 *
 * Purpose:
 *   Communicates with the QMC5883L magnetic compass sensor over the shared I2C bus
 *   (default 7-bit address 0x0D). Includes a continuous background task implementing
 *   tri-state window hysteresis (+1 above +HighMag, 0 within [-LowMag, +LowMag], -1 below -HighMag)
 *   per axis (X, Y, Z) with automatic upstream threshold change notifications ("COMPASS <axis> HIGH/NORMAL/LOW"),
 *   one-shot 3-axis queries, and continuous streaming.
 *
 * Supported Commands:
 *   SetupCompass [addr]
 *     - addr: 7-bit I2C address (default 0x0D / 13).
 *     Initializes QMC5883L sensor registers and starts background monitoring task.
 *     Replies "SETUP COMPASS OK" or "SETUP COMPASS FAIL".
 *
 *   GetCompass / ReadCompass
 *     Reads current instantaneous magnetic field counts for X, Y, and Z axes.
 *     Replies "COMPASS <x> <y> <z>" (e.g. "COMPASS -245 1180 342").
 *
 *   StreamCompass <0|1>
 *     - 1 to enable, 0 to disable continuous streaming of readings every 100ms.
 *     Replies "COMPASS STREAM ON" or "COMPASS STREAM OFF".
 *
 *   LowMag <val>
 *     - val: Magnetic threshold count applied to all three axes X, Y, Z (default 1000).
 *     Sets positive (+val) and negative (-val) lower bound for hysteresis.
 *
 *   HighMag <val>
 *     - val: Magnetic threshold count applied to all three axes X, Y, Z (default 2000).
 *     Sets positive (+val) and negative (-val) upper bound for hysteresis.
 *
 *   LowMagX <val> / LowMagY <val> / LowMagZ <val>
 *     - val: Magnetic threshold count applied specifically to the X, Y, or Z axis.
 *
 *   HighMagX <val> / HighMagY <val> / HighMagZ <val>
 *     - val: Magnetic threshold count applied specifically to the X, Y, or Z axis.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "UpstreamQ.h"
#include "mesh_zigbee.h"
#include "mesh_serial.h"
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
static SemaphoreHandle_t s_compass_mutex = NULL;

// Hysteresis thresholds [LowMag, HighMag] for X, Y, Z axes (default: [1000, 2000])
// Supports both positive-going [LowMag, HighMag] and negative-going -[LowMag, HighMag]
volatile int16_t LowMag[3]  = {1000, 1000, 1000};
volatile int16_t HighMag[3] = {2000, 2000, 2000};
static bool g_stream_compass = false;

// Comparator state per axis: -1 (below -HighMag), 0 (within [-LowMag, +LowMag]), +1 (above +HighMag)
static volatile int8_t s_axis_state[3]    = {0, 0, 0};
static volatile int8_t s_axis_reported[3] = {0, 0, 0};
static volatile int16_t s_axis_last_snapshot[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

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
    if (s_compass_mutex == NULL) {
        s_compass_mutex = xSemaphoreCreateMutex();
    }

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

    if (s_compass_mutex) xSemaphoreTake(s_compass_mutex, portMAX_DELAY);

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
            if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);
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
        if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);
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
        if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);
        ESP_LOGE(TAG, "Failed to configure Control Register 1: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. Control Register 2: Pointer roll-over enabled (0x40), interrupt disabled
    ret = compass_write_reg(QMC5883L_REG_CTRL2, 0x40);
    if (ret != ESP_OK) {
        if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);
        ESP_LOGE(TAG, "Failed to configure Control Register 2: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);

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

    if (s_compass_mutex) xSemaphoreTake(s_compass_mutex, portMAX_DELAY);
    esp_err_t ret = i2c_master_transmit_receive(s_dev_handle, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(100));
    if (s_compass_mutex) xSemaphoreGive(s_compass_mutex);

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

static int8_t evaluate_axis_hysteresis(int8_t cur_state, int16_t val, int16_t low_mag, int16_t high_mag)
{
    int8_t next_state = cur_state;

    if (cur_state == 0) {
        if (val > high_mag) {
            next_state = 1;
        } else if (val < -high_mag) {
            next_state = -1;
        }
    } else if (cur_state == 1) {
        if (val < -high_mag) {
            next_state = -1;
        } else if (val < low_mag) {
            next_state = 0;
        }
    } else if (cur_state == -1) {
        if (val > high_mag) {
            next_state = 1;
        } else if (val > -low_mag) {
            next_state = 0;
        }
    }

    return next_state;
}

void compass_task(void *arg)
{
    const char axis_names[3] = {'X', 'Y', 'Z'};
    int16_t vals[3] = {0, 0, 0};
    uint8_t print_counter = 0;
    char localMsgBuf[64];

    while (1) {
        if (compass_is_initialized() && i2c_manager_is_initialized()) {
            if (compass_read_mag(&vals[0], &vals[1], &vals[2]) == ESP_OK) {
                if (g_stream_compass) {
                    if (++print_counter >= 6) {
                        mesh_serial_writef("COMPASS: %d,%d,%d\r\n", vals[0], vals[1], vals[2]);
                        print_counter = 0;
                    }
                }

                for (int i = 0; i < 3; i++) {
                    int16_t l_mag = LowMag[i];
                    int16_t h_mag = HighMag[i];

                    // Step 1: Check return-to-zero hysteresis crossings (+LowMag or -LowMag) first
                    // so fast polarity reversals (+1 -> -1 or -1 -> +1) never skip reporting the LowMag crossing.
                    if (s_axis_state[i] == 1 && vals[i] < l_mag) {
                        s_axis_state[i] = 0;
                        s_axis_last_snapshot[i][0] = vals[0];
                        s_axis_last_snapshot[i][1] = vals[1];
                        s_axis_last_snapshot[i][2] = vals[2];
                        ESP_LOGI(TAG, "Mag%c +LowMag crossing -> state=0 (val=%d < +%d) [%d %d %d]",
                                 axis_names[i], vals[i], l_mag, vals[0], vals[1], vals[2]);
                        if (s_axis_reported[i] != 0) {
                            snprintf(localMsgBuf, sizeof(localMsgBuf), "Mag%c 0 %d %d %d",
                                     axis_names[i], vals[0], vals[1], vals[2]);
                            if (SendTheMessage(localMsgBuf)) {
                                s_axis_reported[i] = 0;
                            }
                        }
                    } else if (s_axis_state[i] == -1 && vals[i] > -l_mag) {
                        s_axis_state[i] = 0;
                        s_axis_last_snapshot[i][0] = vals[0];
                        s_axis_last_snapshot[i][1] = vals[1];
                        s_axis_last_snapshot[i][2] = vals[2];
                        ESP_LOGI(TAG, "Mag%c -LowMag crossing -> state=0 (val=%d > -%d) [%d %d %d]",
                                 axis_names[i], vals[i], l_mag, vals[0], vals[1], vals[2]);
                        if (s_axis_reported[i] != 0) {
                            snprintf(localMsgBuf, sizeof(localMsgBuf), "Mag%c 0 %d %d %d",
                                     axis_names[i], vals[0], vals[1], vals[2]);
                            if (SendTheMessage(localMsgBuf)) {
                                s_axis_reported[i] = 0;
                            }
                        }
                    }

                    // Step 2: Check outer threshold crossings (+HighMag or -HighMag) from state 0
                    int8_t next_state = evaluate_axis_hysteresis(s_axis_state[i], vals[i], l_mag, h_mag);
                    if (next_state != s_axis_state[i]) {
                        s_axis_state[i] = next_state;
                        s_axis_last_snapshot[i][0] = vals[0];
                        s_axis_last_snapshot[i][1] = vals[1];
                        s_axis_last_snapshot[i][2] = vals[2];
                        ESP_LOGI(TAG, "Mag%c HighMag crossing -> state=%d (val=%d, LowMag=%d, HighMag=%d) [%d %d %d]",
                                 axis_names[i], next_state, vals[i], l_mag, h_mag, vals[0], vals[1], vals[2]);
                    }

                    if (s_axis_reported[i] != s_axis_state[i]) {
                        snprintf(localMsgBuf, sizeof(localMsgBuf), "Mag%c %d %d %d %d",
                                 axis_names[i], s_axis_state[i],
                                 s_axis_last_snapshot[i][0], s_axis_last_snapshot[i][1], s_axis_last_snapshot[i][2]);
                        if (SendTheMessage(localMsgBuf)) {
                            s_axis_reported[i] = s_axis_state[i];
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool compass_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "SetupCompass") == 0 ||
            strcasecmp(cmd, "GetCompass") == 0 ||
            strcasecmp(cmd, "ReadCompass") == 0 ||
            strcasecmp(cmd, "StreamCompass") == 0 ||
            strcasecmp(cmd, "LowMag") == 0 ||
            strcasecmp(cmd, "HighMag") == 0 ||
            strcasecmp(cmd, "LowMagX") == 0 ||
            strcasecmp(cmd, "LowMagY") == 0 ||
            strcasecmp(cmd, "LowMagZ") == 0 ||
            strcasecmp(cmd, "HighMagX") == 0 ||
            strcasecmp(cmd, "HighMagY") == 0 ||
            strcasecmp(cmd, "HighMagZ") == 0);
}

bool compass_execute_cmd(const char *cmd, int16_t value, bool text_message)
{
    if (strcasecmp(cmd, "SetupCompass") == 0) {
        uint8_t addr = (value > 0) ? (uint8_t)value : QMC5883L_DEFAULT_I2C_ADDR;
        if (compass_init(addr) == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                s_axis_state[i] = 0;
                s_axis_reported[i] = 0;
                s_axis_last_snapshot[i][0] = 0;
                s_axis_last_snapshot[i][1] = 0;
                s_axis_last_snapshot[i][2] = 0;
            }
            static bool s_task_created = false;
            if (!s_task_created) {
                xTaskCreate(compass_task, "compass_task", 3072, NULL, 5, NULL);
                s_task_created = true;
                ESP_LOGI(TAG, "Compass comparator task started successfully (Default [LowMag=1000, HighMag=2000]).");
            }
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
    else if (strcasecmp(cmd, "StreamCompass") == 0) {
        g_stream_compass = (value != 0);
        char resp[32];
        snprintf(resp, sizeof(resp), "COMPASS STREAM %s", g_stream_compass ? "ON" : "OFF");
        mesh_zigbee_send_text(resp);
        return true;
    }
    else if (strcasecmp(cmd, "LowMag") == 0) {
        int16_t mag = (value < 0) ? -value : value;
        LowMag[0] = LowMag[1] = LowMag[2] = mag;
        ESP_LOGI(TAG, "LowMag set to %d for X, Y, Z (negative threshold = -%d)", mag, mag);
        if (!text_message) mesh_zigbee_send_text("GOT LOWMAG OK");
        return true;
    }
    else if (strcasecmp(cmd, "HighMag") == 0) {
        int16_t mag = (value < 0) ? -value : value;
        HighMag[0] = HighMag[1] = HighMag[2] = mag;
        ESP_LOGI(TAG, "HighMag set to %d for X, Y, Z (negative threshold = -%d)", mag, mag);
        if (!text_message) mesh_zigbee_send_text("GOT HIGHMAG OK");
        return true;
    }
    else if (strncasecmp(cmd, "LowMag", 6) == 0 && (cmd[6] == 'X' || cmd[6] == 'x' || cmd[6] == 'Y' || cmd[6] == 'y' || cmd[6] == 'Z' || cmd[6] == 'z')) {
        int idx = (cmd[6] == 'X' || cmd[6] == 'x') ? 0 : ((cmd[6] == 'Y' || cmd[6] == 'y') ? 1 : 2);
        int16_t mag = (value < 0) ? -value : value;
        LowMag[idx] = mag;
        ESP_LOGI(TAG, "LowMag[%c] set to %d (negative threshold = -%d)", "XYZ"[idx], mag, mag);
        if (!text_message) mesh_zigbee_send_text("GOT LOWMAG OK");
        return true;
    }
    else if (strncasecmp(cmd, "HighMag", 7) == 0 && (cmd[7] == 'X' || cmd[7] == 'x' || cmd[7] == 'Y' || cmd[7] == 'y' || cmd[7] == 'Z' || cmd[7] == 'z')) {
        int idx = (cmd[7] == 'X' || cmd[7] == 'x') ? 0 : ((cmd[7] == 'Y' || cmd[7] == 'y') ? 1 : 2);
        int16_t mag = (value < 0) ? -value : value;
        HighMag[idx] = mag;
        ESP_LOGI(TAG, "HighMag[%c] set to %d (negative threshold = -%d)", "XYZ"[idx], mag, mag);
        if (!text_message) mesh_zigbee_send_text("GOT HIGHMAG OK");
        return true;
    }
    return false;
}

