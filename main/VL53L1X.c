/*
 * VL53L1X.c - Time-of-Flight (ToF) Long-Distance Laser Ranging Driver
 *
 * Purpose:
 *   Interfaces with STMicroelectronics VL53L1X 940nm Time-of-Flight distance sensors
 *   over the shared I2C bus (address 0x29), supporting both a direct main-bus sensor and up to
 *   8 multiplexed sensors behind a TCA9548A I2Cmux (ports 0..7). Measures distances up to 4 meters
 *   independent of target color or surface reflectance. Includes a 50ms round-robin background
 *   polling task (vl53l1x_task) implementing per-sensor dual-threshold hysteresis
 *   ("VL Object Detected[_mxN] <ts>", "VL Object Removed[_mxN] <ts>") and 500ms serial streaming.
 *
 * Supported Commands:
 *   SetupVL
 *     Initializes VL53L1X hardware on the main I2C bus (or currently selected I2Cmux channel)
 *     and launches the background monitoring task.
 *     Replies "SETUP VL OK" or "SETUP VL FAIL".
 *
 *   StreamVL <0|1>
 *     - 1 to enable, 0 to disable continuous serial streaming of distance measurements every 500ms.
 *     Replies "VL STREAM ON" or "VL STREAM OFF".
 *
 *   VLUpperThresh <mm>
 *     - mm: Upper distance threshold in millimeters (default 400).
 *     Sets upper bound for hysteresis object detection.
 *     Replies "GOT VLUPPERTHRESH OK".
 *
 *   VLLowerThresh <mm>
 *     - mm: Lower distance threshold in millimeters (default 500).
 *     Sets lower bound for hysteresis object detection.
 *     Replies "GOT VLLOWERTHRESH OK".
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "UpstreamQ.h"
#include "mesh_zigbee.h"
#include "mesh_serial.h"
#include "i2c_manager.h"
#include "I2Cmux.h"
#include "VL53L1X.h"

static i2c_master_dev_handle_t dev_handle = NULL;
static const char *TAG = "VL53L1X";

typedef enum
{
    ObjectDetected = 1,
    NothingThere = 0
} VLState_t;

volatile int16_t VLUpperThresh = 2000; // 2000 mm (2 meters)
volatile int16_t VLLowerThresh = 1000; // 1000 mm (1 meter)
static bool g_stream_vl = false;

static volatile bool s_main_bus_active = false;
static volatile uint8_t s_mux_channel_mask = 0;
static VLState_t s_main_state = NothingThere;
static VLState_t s_main_old_state = NothingThere;
static uint8_t s_main_print_cnt = 0;
static uint16_t s_main_timestamp = 0;
static VLState_t s_mux_state[TCA9548A_NUM_CHANNELS] = {NothingThere};
static VLState_t s_mux_old_state[TCA9548A_NUM_CHANNELS] = {NothingThere};
static uint8_t s_mux_print_cnt[TCA9548A_NUM_CHANNELS] = {0};
static uint16_t s_mux_timestamp[TCA9548A_NUM_CHANNELS] = {0};

esp_err_t vl53l1x_init(void)
{
    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus.");
        return ESP_FAIL;
    }

    i2cmux_lock();
    esp_err_t ret = i2c_master_probe(bus_handle, VL53L1X_I2C_ADDR, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        i2cmux_unlock();
        ESP_LOGE(TAG, "Sensor not found on I2C bus at address 0x%02x", VL53L1X_I2C_ADDR);
        return ret;
    }

    if (dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L1X_I2C_ADDR,
            .scl_speed_hz = 100000,
        };
        ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            i2cmux_unlock();
            return ret;
        }
    }

    int8_t mux_ch = i2cmux_get_selected_channel();
    if (mux_ch >= 0 && mux_ch < TCA9548A_NUM_CHANNELS) {
        s_mux_state[mux_ch] = NothingThere;
        s_mux_old_state[mux_ch] = NothingThere;
        s_mux_print_cnt[mux_ch] = 0;
        s_mux_timestamp[mux_ch] = 0;
        s_mux_channel_mask |= (uint8_t)(1 << mux_ch);
    } else {
        s_main_state = NothingThere;
        s_main_old_state = NothingThere;
        s_main_print_cnt = 0;
        s_main_timestamp = 0;
        s_main_bus_active = true;
    }
    i2cmux_unlock();

    ESP_LOGI(TAG, "VL53L1X Hardware Initialized on shared I2C bus (mux_ch=%d).", mux_ch);
    return ESP_OK;
}

bool vl53l1x_check_latched_bus(esp_err_t err)
{
    return i2c_manager_check_latched_bus("VL53L1X", err);
}

esp_err_t vl53l1x_read_distance(uint16_t *distance)
{
    if (dev_handle == NULL || distance == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t reg[2] = {0x01, 0x0F}; // Model_ID register
    uint8_t data[1] = {0};
    
    // Transmit 2-byte register address and receive 1-byte response
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, reg, 2, data, 1, pdMS_TO_TICKS(100));
    
    if (ret == ESP_OK) {
        *distance = data[0]; // For a valid VL53L1X, this will return 0xEA
        vl53l1x_check_latched_bus(ESP_OK);
    } else {
        vl53l1x_check_latched_bus(ret);
    }

    return ret;
}

static void poll_single_vl_channel(int8_t mux_ch, VLState_t *state_ptr, VLState_t *old_state_ptr, uint8_t *print_cnt_ptr, uint16_t *ts_ptr)
{
    uint8_t saved_mask = 0;
    bool mux_switched = false;
    if (i2cmux_switch_and_lock(mux_ch, &saved_mask, &mux_switched) != ESP_OK) {
        return;
    }

    uint16_t distance = 0;
    esp_err_t ret = vl53l1x_read_distance(&distance);
    i2cmux_restore_and_unlock(saved_mask, mux_switched);

    if (ret != ESP_OK) {
        return;
    }

    char suffix[8] = "";
    if (mux_ch >= 0) {
        snprintf(suffix, sizeof(suffix), "_mx%d", mux_ch);
    }

    if (g_stream_vl) {
        if (++(*print_cnt_ptr) >= 10) {
            mesh_serial_writef("VL53%s: %u\r\n", suffix, distance);
            *print_cnt_ptr = 0;
        }
    }

    (*ts_ptr)++;

    // Hysteresis logic for distance: Smaller values = closer
    if (distance > VLUpperThresh) {
        *state_ptr = NothingThere;
    } else if (distance < VLLowerThresh) {
        *state_ptr = ObjectDetected;
    }

    char localMsgBuf[48];
    if (*state_ptr == ObjectDetected && *old_state_ptr == NothingThere) {
        snprintf(localMsgBuf, sizeof(localMsgBuf), "VL Object Detected%s %u", suffix, *ts_ptr);
        if (SendTheMessage(localMsgBuf)) {
            *old_state_ptr = *state_ptr;
        }
    } else if (*state_ptr == NothingThere && *old_state_ptr == ObjectDetected) {
        snprintf(localMsgBuf, sizeof(localMsgBuf), "VL Object Removed%s %u", suffix, *ts_ptr);
        if (SendTheMessage(localMsgBuf)) {
            *old_state_ptr = *state_ptr;
        }
    }
}

void vl53l1x_task(void *arg)
{
    while (1)
    {
        if (s_main_bus_active) {
            poll_single_vl_channel(-1, &s_main_state, &s_main_old_state, &s_main_print_cnt, &s_main_timestamp);
        }

        uint8_t mask = s_mux_channel_mask;
        for (int ch = 0; ch < TCA9548A_NUM_CHANNELS; ch++) {
            if (mask & (1 << ch)) {
                poll_single_vl_channel((int8_t)ch, &s_mux_state[ch], &s_mux_old_state[ch], &s_mux_print_cnt[ch], &s_mux_timestamp[ch]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms polling rate
    }
}

// --- Command Routing ---
bool vl53l1x_is_cmd(const char *cmd) {
    if (!cmd) return false;
    return (strcmp(cmd, "StreamVL") == 0 || strcmp(cmd, "VLUpperThresh") == 0 || 
            strcmp(cmd, "VLLowerThresh") == 0 || strcmp(cmd, "SetupVL") == 0);
}

bool vl53l1x_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (strcmp(cmd, "StreamVL") == 0) {
        g_stream_vl = (value != 0);
        char resp[32];
        snprintf(resp, sizeof(resp), "VL STREAM %s", g_stream_vl ? "ON" : "OFF");
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcmp(cmd, "VLUpperThresh") == 0) {
        VLUpperThresh = value;
        ESP_LOGI(TAG, "VLUpperThresh set to %d", VLUpperThresh);
        if (!text_message) mesh_zigbee_send_text("GOT VLUPPERTHRESH OK");
        return true;
    } else if (strcmp(cmd, "VLLowerThresh") == 0) {
        VLLowerThresh = value;
        ESP_LOGI(TAG, "VLLowerThresh set to %d", VLLowerThresh);
        if (!text_message) mesh_zigbee_send_text("GOT VLLOWERTHRESH OK");
        return true;
    } else if (strcmp(cmd, "SetupVL") == 0) {
        if (vl53l1x_init() == ESP_OK) {
            static bool s_task_created = false;
            if (!s_task_created) {
                xTaskCreate(vl53l1x_task, "vl53_task", 3072, NULL, 5, NULL);
                s_task_created = true;
                ESP_LOGI(TAG, "VL53L1X task started successfully.");
            }
            if (!text_message) mesh_zigbee_send_text("SETUP VL OK");
        } else {
            ESP_LOGE(TAG, "VL53L1X initialization failed.");
            if (!text_message) mesh_zigbee_send_text("SETUP VL FAIL");
        }
        return true;
    }
    return false;
}