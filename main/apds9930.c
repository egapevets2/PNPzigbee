// File: apds9930.c

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

#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_SCL_IO 23
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// #define _9930_DEBUG_PRINT_ENABLE

i2c_master_bus_handle_t g_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static const char *TAG = "APDS9930";

// Variables now owned natively by the driver!
volatile int16_t Npulses = 8;
volatile int16_t UpperThresh = 200;
volatile int16_t LowerThresh = 150;
static bool g_stream_prox = false;

esp_err_t apds9930_config(void)
{
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C device not initialized yet.");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ATIME = 0xff;         // 2.7 ms – minimum ALS integration time
    uint8_t WTIME = 0xff;         // 2.7 ms – minimum Wait time
    uint8_t PTIME = 0xff;         // 2.7 ms – minimum Prox integration time
    uint8_t PPULSE = Npulses - 1; // Uses your updated global Npulses
    uint8_t PDRIVE = 0;           // 100mA of LED Power
    uint8_t PDIODE = 0x20;        // CH1 Diode
    uint8_t PGAIN = 10;           // Prox gain
    uint8_t AGAIN = 0;            // 1x ALS gain

    uint8_t WEN = 8; // Enable Wait
    uint8_t PEN = 4; // Enable Prox
    uint8_t AEN = 2; // Enable ALS
    uint8_t PON = 1; // Enable Power On

    uint8_t setup[][2] = {
        {0x80 | 0x00, 0},
        {0x80 | 0x01, ATIME},
        {0x80 | 0x02, PTIME},
        {0x80 | 0x03, WTIME},
        {0x80 | 0x0E, PPULSE},
        {0x80 | 0x0F, (uint8_t)(PDRIVE | PDIODE | PGAIN | AGAIN)},
        {0x80 | 0x00, (uint8_t)(WEN | PEN | AEN | PON)}};

    int num_commands = sizeof(setup) / sizeof(setup[0]);

    for (int i = 0; i < num_commands; i++)
    {
        uint8_t buf[2] = {setup[i][0], setup[i][1]};
        esp_err_t ret = i2c_master_transmit(dev_handle, buf, 2, -1);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write setup register 0x%02x", setup[i][0] & ~0x80);
            return ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(12));
    ESP_LOGI(TAG, "Sensor configured successfully with Npulses=%d.", Npulses);
    return ESP_OK;
}

i2c_master_bus_handle_t get_shared_i2c_bus(void)
{
    if (g_i2c_bus_handle == NULL) {
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_NUM,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        i2c_new_master_bus(&i2c_mst_config, &g_i2c_bus_handle);
    }
    return g_i2c_bus_handle;
}

esp_err_t apds9930_init(void)
{
    i2c_master_bus_handle_t bus_handle = get_shared_i2c_bus();
    if (bus_handle == NULL) return ESP_FAIL;
    esp_err_t ret;

    ret = i2c_master_probe(bus_handle, APDS9930_I2C_ADDR, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensor not found on I2C bus at address 0x%02x", APDS9930_I2C_ADDR);
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = APDS9930_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "I2C Hardware Initialized. Pushing initial config...");

    // Call the config function once during boot
    return apds9930_config();
}

esp_err_t apds9930_read_proximity(uint16_t *proximity)
{
    uint8_t reg = 0xA0 | 0x18;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, -1);

    *proximity = data[0] + (256 * data[1]);

    return ret;
}

void proximity_task(void *arg)
{
    uint16_t proximity = 0;
    uint16_t timestamp = 0;
    uint8_t print_counter = 0;

    typedef enum
    {
        CarDetected = 1,
        NothingThere = 0
    } ProximityState_t;

    ProximityState_t state = NothingThere;
    ProximityState_t OLDstate = NothingThere;

    // Local buffer for formatting messages
    char localMsgBuf[32]; 

    while (1)
    {
        if (apds9930_read_proximity(&proximity) == ESP_OK)
        {
            if (g_stream_prox)
            {
                if (++print_counter >= 10)
                {
                    mesh_serial_writef("PROX: %u\r\n", proximity);
                    print_counter = 0;
                }
            }

            timestamp++;
            if (proximity > UpperThresh)
            {
                state = CarDetected;
            }
            else if (proximity < LowerThresh)
            {
                state = NothingThere;
            }

            if (state == CarDetected && OLDstate == NothingThere)
            {
                snprintf(localMsgBuf, sizeof(localMsgBuf), "Car Detected %d", timestamp);
                SendTheMessage(localMsgBuf);
            }
            else if (state == NothingThere && OLDstate == CarDetected)
            {
                snprintf(localMsgBuf, sizeof(localMsgBuf), "Car removed %d", timestamp);
                SendTheMessage(localMsgBuf);
            }

            OLDstate = state;

#ifdef _9930_DEBUG_PRINT_ENABLE
            printf("%u,%d,%d,%d\n", proximity, state, UpperThresh, LowerThresh);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Command Routing ---
bool proximity_is_cmd(const char *cmd) {
    if (!cmd) return false;
    return (strcmp(cmd, "StreamProx") == 0 || strcmp(cmd, "Npulses") == 0 || strcmp(cmd, "UpperThresh") == 0 || 
            strcmp(cmd, "LowerThresh") == 0 || strcmp(cmd, "SetupProximity") == 0);
}

bool proximity_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (strcmp(cmd, "StreamProx") == 0) {
        g_stream_prox = (value != 0);
        char resp[32];
        snprintf(resp, sizeof(resp), "PROX STREAM %s", g_stream_prox ? "ON" : "OFF");
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcmp(cmd, "Npulses") == 0) {
        Npulses = value;
        ESP_LOGI(TAG, "Npulses set to %d. Reconfiguring sensor...", Npulses);
        apds9930_config();
        if (!text_message) mesh_zigbee_send_text("GOT NPULSES OK");
        return true;
    } else if (strcmp(cmd, "UpperThresh") == 0) {
        UpperThresh = value;
        ESP_LOGI(TAG, "UpperThresh set to %d", UpperThresh);
        if (!text_message) mesh_zigbee_send_text("GOT UPPERTHRESH OK");
        return true;
    } else if (strcmp(cmd, "LowerThresh") == 0) {
        LowerThresh = value;
        ESP_LOGI(TAG, "LowerThresh set to %d", LowerThresh);
        if (!text_message) mesh_zigbee_send_text("GOT LOWERTHRESH OK");
        return true;
    } else if (strcmp(cmd, "SetupProximity") == 0) {
        if (apds9930_init() == ESP_OK) {
            xTaskCreate(proximity_task, "prox_task", 3072, NULL, 5, NULL);
            ESP_LOGI(TAG, "Proximity task started successfully.");
            if (!text_message) mesh_zigbee_send_text("SETUP PROX OK");
        } else {
            ESP_LOGE(TAG, "APDS9930 initialization failed.");
            if (!text_message) mesh_zigbee_send_text("SETUP PROX FAIL");
        }
        return true;
    }
    return false;
}