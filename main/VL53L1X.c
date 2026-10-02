// VL53L1X laser distance sensor
/*
The VL53L1X is an advanced, miniature Time-of-Flight (ToF) distance 
sensor from STMicroelectronics. It utilizes an invisible, eye-safe 
940nm laser to measure absolute distances up to 4 meters, entirely 
independent of the target's color or reflectivity. 
It is widely used in robotics, drones, and smart home automation


This sensor shares the I2C bus with any other i2c devices, such as
the apds9930.c



Supports these commands:







*/



#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "apds9930.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "UpstreamQ.h"
#include "mesh_zigbee.h"
#include "mesh_serial.h"
#include "VL53L1X.h"

static i2c_master_dev_handle_t dev_handle = NULL;
static const char *TAG = "VL53L1X";

volatile int16_t VLUpperThresh = 2000; // 2000 mm (2 meters)
volatile int16_t VLLowerThresh = 1000; // 1000 mm (1 meter)
static bool g_stream_vl = false;

esp_err_t vl53l1x_init(void)
{
    i2c_master_bus_handle_t bus_handle = get_shared_i2c_bus();
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus from APDS9930 driver.");
        return ESP_FAIL;
    }

    esp_err_t ret = i2c_master_probe(bus_handle, VL53L1X_I2C_ADDR, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor not found on I2C bus at address 0x%02x", VL53L1X_I2C_ADDR);
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L1X_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "VL53L1X Hardware Initialized on shared I2C bus.");

    // Note: A full VL53L1X driver requires downloading firmware and configuration
    // parameters to the sensor here. For a complete production implementation, 
    // you must integrate the STMicroelectronics Ultra Lite Driver (ULD) API.
    // Example: VL53L1X_SensorInit(dev);
    
    return ESP_OK;
}

esp_err_t vl53l1x_read_distance(uint16_t *distance)
{
    if (dev_handle == NULL) return ESP_ERR_INVALID_STATE;

    // NOTE: True distance reading requires the STMicroelectronics ULD API.
    // This function is currently a structural placeholder that reads the Model ID 
    // register to prove the I2C communication is successfully shared and working.

    uint8_t reg[2] = {0x01, 0x0F}; // Model_ID register
    uint8_t data[1] = {0};
    
    // Transmit 2-byte register address and receive 1-byte response
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, reg, 2, data, 1, -1);
    
    if (ret == ESP_OK) {
        *distance = data[0]; // For a valid VL53L1X, this will return 0xEA
    }

    return ret;
}

void vl53l1x_task(void *arg)
{
    uint16_t distance = 0;
    uint16_t timestamp = 0;
    uint8_t print_counter = 0;

    typedef enum
    {
        ObjectDetected = 1,
        NothingThere = 0
    } VLState_t;

    VLState_t state = NothingThere;
    VLState_t OLDstate = NothingThere;

    char localMsgBuf[32]; 

    while (1)
    {
        if (vl53l1x_read_distance(&distance) == ESP_OK)
        {
            if (g_stream_vl)
            {
                if (++print_counter >= 10)
                {
                    mesh_serial_writef("VL53: %u\r\n", distance);
                    print_counter = 0;
                }
            }

            timestamp++;
            
            // Hysteresis logic for distance: Smaller values = closer
            if (distance > VLUpperThresh)
            {
                state = NothingThere;
            }
            else if (distance < VLLowerThresh)
            {
                state = ObjectDetected;
            }

            if (state == ObjectDetected && OLDstate == NothingThere)
            {
                snprintf(localMsgBuf, sizeof(localMsgBuf), "VL Object Detected %d", timestamp);
                SendTheMessage(localMsgBuf);
            }
            else if (state == NothingThere && OLDstate == ObjectDetected)
            {
                snprintf(localMsgBuf, sizeof(localMsgBuf), "VL Object Removed %d", timestamp);
                SendTheMessage(localMsgBuf);
            }

            OLDstate = state;
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
            xTaskCreate(vl53l1x_task, "vl53_task", 3072, NULL, 5, NULL);
            ESP_LOGI(TAG, "VL53L1X task started successfully.");
            if (!text_message) mesh_zigbee_send_text("SETUP VL OK");
        } else {
            ESP_LOGE(TAG, "VL53L1X initialization failed.");
            if (!text_message) mesh_zigbee_send_text("SETUP VL FAIL");
        }
        return true;
    }
    return false;
}