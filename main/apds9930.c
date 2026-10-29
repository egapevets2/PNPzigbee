/*
 * apds9930.c - APDS-9930 Digital Proximity & Ambient Light Sensor Driver
 *
 * Purpose:
 *   Interfaces with Broadcom / Avago APDS-9930 optical proximity sensors over the
 *   shared I2C bus (address 0x39), supporting both a direct main-bus sensor and up to 8
 *   multiplexed sensors behind a TCA9548A I2Cmux (ports 0..7). Features 20ms round-robin
 *   background polling (proximity_task), per-sensor dual-threshold hysteresis event generation
 *   ("Car Detected[_mxN] <val>", "Car removed[_mxN] <val>") sent upstream to the coordinator,
 *   one-shot proximity reads, IR LED emitter pulse count adjustments across all initialized
 *   sensors, and 200ms continuous serial streaming.
 *
 * Supported Commands:
 *   SetupProximity
 *     Initializes APDS-9930 I2C registers on the main bus (or currently selected I2Cmux channel)
 *     and starts the background monitoring task.
 *     Replies "SETUP PROX OK" or "SETUP PROX FAIL".
 *
 *   ReadProximity / ReadProx
 *     Reads instantaneous 16-bit proximity measurement count on the main bus or active I2Cmux channel.
 *     Replies "PROXval[_mxN] <val>" (e.g. "PROXval 420" or "PROXval_mx0 420"), "PROX NOT READY",
 *     or "PROXval ERROR".
 *
 *   StreamProx <0|1>
 *     - 1 to enable, 0 to disable continuous serial streaming of proximity values every 200ms.
 *     Replies "PROX STREAM ON" or "PROX STREAM OFF".
 *
 *   Npulses <count>
 *     - count: Number of IR LED proximity pulses (1..255, default 8).
 *     Reconfigures optical emitter pulse count (register PPULSE) on all active APDS-9930 instances.
 *     Replies "GOT NPULSES OK".
 *
 *   ProxHighThrsh <val> / ProxHighThresh <val> / UpperThresh <val>
 *     - val: Proximity count threshold (default 500).
 *     Sets upper threshold for hysteresis object detection ("Car Detected[_mxN] <val>").
 *     Replies "GOT PROXHIGHTHRSH OK".
 *
 *   ProxLowThrsh <val> / ProxLowThresh <val> / LowerThresh <val>
 *     - val: Proximity count threshold (default 400).
 *     Sets lower threshold for hysteresis object removal ("Car removed[_mxN] <val>").
 *     Replies "GOT PROXLOWTHRSH OK".
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
#include "i2c_manager.h"
#include "I2Cmux.h"

#define I2C_MASTER_FREQ_HZ 100000

// #define _9930_DEBUG_PRINT_ENABLE

static i2c_master_dev_handle_t dev_handle = NULL;
static const char *TAG = "APDS9930";

typedef enum
{
    CarDetected = 1,
    NothingThere = 0
} ProximityState_t;

// Variables now owned natively by the driver!
volatile int16_t Npulses = 8;
volatile int16_t UpperThresh = 500;
volatile int16_t LowerThresh = 200;
static bool g_stream_prox = false;

// Multi-sensor I2Cmux tracking (main bus + up to 8 TCA9548A mux ports 0..7)
static volatile bool s_main_bus_active = false;
static volatile uint8_t s_mux_channel_mask = 0;
static ProximityState_t s_main_state = NothingThere;
static ProximityState_t s_main_old_state = NothingThere;
static uint8_t s_main_print_cnt = 0;
static ProximityState_t s_mux_state[TCA9548A_NUM_CHANNELS] = {NothingThere};
static ProximityState_t s_mux_old_state[TCA9548A_NUM_CHANNELS] = {NothingThere};
static uint8_t s_mux_print_cnt[TCA9548A_NUM_CHANNELS] = {0};

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
        esp_err_t ret = i2c_master_transmit(dev_handle, buf, 2, pdMS_TO_TICKS(100));
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write setup register 0x%02x: %s", setup[i][0] & ~0x80, esp_err_to_name(ret));
            apds9930_check_latched_bus(ret);
            return ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(12));
    ESP_LOGI(TAG, "Sensor configured successfully with Npulses=%d.", Npulses);
    return ESP_OK;
}

esp_err_t apds9930_init(void)
{
    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus_handle = i2c_manager_get_bus();
    if (bus_handle == NULL) return ESP_FAIL;
    esp_err_t ret;

    i2cmux_lock();
    ret = i2c_master_probe(bus_handle, APDS9930_I2C_ADDR, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        i2cmux_unlock();
        ESP_LOGE(TAG, "Sensor not found on I2C bus at address 0x%02x: %s", APDS9930_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }

    if (dev_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = APDS9930_I2C_ADDR,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };
        ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) {
            i2cmux_unlock();
            return ret;
        }
    }

    ESP_LOGI(TAG, "I2C Hardware Initialized (mux_ch=%d). Pushing initial config...", i2cmux_get_selected_channel());

    ret = apds9930_config();
    if (ret == ESP_OK) {
        int8_t mux_ch = i2cmux_get_selected_channel();
        if (mux_ch >= 0 && mux_ch < TCA9548A_NUM_CHANNELS) {
            s_mux_state[mux_ch] = NothingThere;
            s_mux_old_state[mux_ch] = NothingThere;
            s_mux_print_cnt[mux_ch] = 0;
            s_mux_channel_mask |= (uint8_t)(1 << mux_ch);
        } else {
            s_main_state = NothingThere;
            s_main_old_state = NothingThere;
            s_main_print_cnt = 0;
            s_main_bus_active = true;
        }
    }
    i2cmux_unlock();
    return ret;
}

bool apds9930_check_latched_bus(esp_err_t err)
{
    return i2c_manager_check_latched_bus("APDS9930", err);
}

esp_err_t apds9930_read_proximity(uint16_t *proximity)
{
    if (dev_handle == NULL || proximity == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t reg = 0xA0 | 0x18;
    uint8_t data[2] = {0};
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, pdMS_TO_TICKS(100));

    if (ret != ESP_OK) {
        apds9930_check_latched_bus(ret);
        return ret;
    }
    apds9930_check_latched_bus(ESP_OK);

    *proximity = data[0] + (256 * data[1]);

    return ret;
}

static void poll_single_proximity_channel(int8_t mux_ch, ProximityState_t *state_ptr, ProximityState_t *old_state_ptr, uint8_t *print_cnt_ptr)
{
    uint8_t saved_mask = 0;
    bool mux_switched = false;
    if (i2cmux_switch_and_lock(mux_ch, &saved_mask, &mux_switched) != ESP_OK) {
        return;
    }

    uint16_t proximity = 0;
    esp_err_t ret = apds9930_read_proximity(&proximity);
    i2cmux_restore_and_unlock(saved_mask, mux_switched);

    if (ret != ESP_OK) {
        return;
    }

    char suffix[8] = "";
    if (mux_ch >= 0) {
        snprintf(suffix, sizeof(suffix), "_mx%d", mux_ch);
    }

    if (g_stream_prox) {
        if (++(*print_cnt_ptr) >= 10) {
            mesh_serial_writef("PROX%s: %u\r\n", suffix, proximity);
            *print_cnt_ptr = 0;
        }
    }

    if (proximity > UpperThresh) {
        *state_ptr = CarDetected;
    } else if (proximity < LowerThresh) {
        *state_ptr = NothingThere;
    }

    char localMsgBuf[48];
    if (*state_ptr == CarDetected && *old_state_ptr == NothingThere) {
        snprintf(localMsgBuf, sizeof(localMsgBuf), "Car Detected%s %u", suffix, proximity);
        if (SendTheMessage(localMsgBuf)) {
            *old_state_ptr = *state_ptr;
        }
    } else if (*state_ptr == NothingThere && *old_state_ptr == CarDetected) {
        snprintf(localMsgBuf, sizeof(localMsgBuf), "Car removed%s %u", suffix, proximity);
        if (SendTheMessage(localMsgBuf)) {
            *old_state_ptr = *state_ptr;
        }
    }

#ifdef _9930_DEBUG_PRINT_ENABLE
    printf("%u,%d,%d,%d\n", proximity, *state_ptr, UpperThresh, LowerThresh);
#endif
}

void proximity_task(void *arg)
{
    while (1)
    {
        if (s_main_bus_active) {
            poll_single_proximity_channel(-1, &s_main_state, &s_main_old_state, &s_main_print_cnt);
        }

        uint8_t mask = s_mux_channel_mask;
        for (int ch = 0; ch < TCA9548A_NUM_CHANNELS; ch++) {
            if (mask & (1 << ch)) {
                poll_single_proximity_channel((int8_t)ch, &s_mux_state[ch], &s_mux_old_state[ch], &s_mux_print_cnt[ch]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Command Routing ---
bool proximity_is_cmd(const char *cmd) {
    if (!cmd) return false;
    return (strcasecmp(cmd, "StreamProx") == 0 ||
            strcasecmp(cmd, "Npulses") == 0 ||
            strcasecmp(cmd, "ProxHighThrsh") == 0 ||
            strcasecmp(cmd, "ProxLowThrsh") == 0 ||
            strcasecmp(cmd, "ProxHighThresh") == 0 ||
            strcasecmp(cmd, "ProxLowThresh") == 0 ||
            strcasecmp(cmd, "UpperThresh") == 0 ||
            strcasecmp(cmd, "LowerThresh") == 0 ||
            strcasecmp(cmd, "SetupProximity") == 0 ||
            strcasecmp(cmd, "ReadProximity") == 0 ||
            strcasecmp(cmd, "ReadProx") == 0);
}

bool proximity_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (strcasecmp(cmd, "StreamProx") == 0) {
        g_stream_prox = (value != 0);
        char resp[32];
        snprintf(resp, sizeof(resp), "PROX STREAM %s", g_stream_prox ? "ON" : "OFF");
        mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "Npulses") == 0) {
        Npulses = value;
        ESP_LOGI(TAG, "Npulses set to %d. Reconfiguring sensor(s)...", Npulses);
        if (s_main_bus_active) {
            uint8_t saved_mask = 0;
            bool switched = false;
            if (i2cmux_switch_and_lock(-1, &saved_mask, &switched) == ESP_OK) {
                apds9930_config();
                i2cmux_restore_and_unlock(saved_mask, switched);
            }
        }
        uint8_t mask = s_mux_channel_mask;
        for (int ch = 0; ch < TCA9548A_NUM_CHANNELS; ch++) {
            if (mask & (1 << ch)) {
                uint8_t saved_mask = 0;
                bool switched = false;
                if (i2cmux_switch_and_lock((int8_t)ch, &saved_mask, &switched) == ESP_OK) {
                    apds9930_config();
                    i2cmux_restore_and_unlock(saved_mask, switched);
                }
            }
        }
        if (!text_message) mesh_zigbee_send_text("GOT NPULSES OK");
        return true;
    } else if (strcasecmp(cmd, "ProxHighThrsh") == 0 ||
               strcasecmp(cmd, "ProxHighThresh") == 0 ||
               strcasecmp(cmd, "UpperThresh") == 0) {
        UpperThresh = value;
        ESP_LOGI(TAG, "ProxHighThrsh (UpperThresh) set to %d", UpperThresh);
        if (!text_message) mesh_zigbee_send_text("GOT PROXHIGHTHRSH OK");
        return true;
    } else if (strcasecmp(cmd, "ProxLowThrsh") == 0 ||
               strcasecmp(cmd, "ProxLowThresh") == 0 ||
               strcasecmp(cmd, "LowerThresh") == 0) {
        LowerThresh = value;
        ESP_LOGI(TAG, "ProxLowThrsh (LowerThresh) set to %d", LowerThresh);
        if (!text_message) mesh_zigbee_send_text("GOT PROXLOWTHRSH OK");
        return true;
    } else if (strcasecmp(cmd, "ReadProximity") == 0 || strcasecmp(cmd, "ReadProx") == 0) {
        if (dev_handle == NULL) {
            mesh_zigbee_send_text("PROX NOT READY");
            return true;
        }
        i2cmux_lock();
        int8_t mux_ch = i2cmux_get_selected_channel();
        uint16_t prox = 0;
        esp_err_t rd_err = apds9930_read_proximity(&prox);
        i2cmux_unlock();

        char suffix[8] = "";
        if (mux_ch >= 0) {
            snprintf(suffix, sizeof(suffix), "_mx%d", mux_ch);
        }
        char resp[32];
        if (rd_err == ESP_OK) {
            snprintf(resp, sizeof(resp), "PROXval%s %u", suffix, prox);
        } else {
            snprintf(resp, sizeof(resp), "PROXval%s ERROR", suffix);
        }
        mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "SetupProximity") == 0) {
        if (apds9930_init() == ESP_OK) {
            static bool s_task_created = false;
            if (!s_task_created) {
                xTaskCreate(proximity_task, "prox_task", 3072, NULL, 5, NULL);
                s_task_created = true;
                ESP_LOGI(TAG, "Proximity task started successfully.");
            }
            mesh_zigbee_send_text("SETUP PROX OK");
        } else {
            ESP_LOGE(TAG, "APDS9930 initialization failed.");
            mesh_zigbee_send_text("SETUP PROX FAIL");
        }
        return true;
    }
    return false;
}
