#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "UpstreamQ.h"
#include "adc.h"
#include "mesh_zigbee.h"
#include "mesh_serial.h"

static const char *TAG = "ADC_CTRL";

// Define the struct based on requirements
typedef struct {
    bool active;
    int iGPIO;
    int LowerThresh;
    int UpperThresh;
    bool Comparator;
    const char* name;
} adc_config_t;

static adc_config_t g_adc[3] = {
    { .active = false, .iGPIO = 2,  .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .name = "A0" },
    { .active = false, .iGPIO = 3,  .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .name = "A1" },
    { .active = false, .iGPIO = 10, .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .name = "A2" }
};

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static bool adc_task_created = false;
static bool g_stream_serial = false;

// Helper to map GPIO to ESP32-C6 ADC1 Channels
static adc_channel_t get_adc_channel(int gpio) {
    switch (gpio) {
        case 0:  return ADC_CHANNEL_0;
        case 1:  return ADC_CHANNEL_1;
        case 2:  return ADC_CHANNEL_2;
        case 3:  return ADC_CHANNEL_3;
        case 14: return ADC_CHANNEL_4;
        case 15: return ADC_CHANNEL_5;
        case 10: return ADC_CHANNEL_6;
        default: return ADC_CHANNEL_3;
    }
}

// The task that implements the comparator with hysteresis
static void adc_task(void *arg) {
    int current_idx = 0;
    while (1) {
        if (g_adc[current_idx].active) {
            int raw_val = 0;
            adc_oneshot_read(adc1_handle, get_adc_channel(g_adc[current_idx].iGPIO), &raw_val);

            bool old_state = g_adc[current_idx].Comparator;

            if (g_stream_serial) {
                mesh_serial_writef("ADC%s: %d\r\n", g_adc[current_idx].name, raw_val);
            }

            // Hysteresis logic
            if (raw_val > g_adc[current_idx].UpperThresh) {
                g_adc[current_idx].Comparator = true;
            } else if (raw_val < g_adc[current_idx].LowerThresh) {
                g_adc[current_idx].Comparator = false;
            }

            // If state changes, notify the coordinator
            if (old_state != g_adc[current_idx].Comparator) {
                char msg[32];
                snprintf(msg, sizeof(msg), "ADCcomparator%s %d", g_adc[current_idx].name, g_adc[current_idx].Comparator ? 1 : 0);
                SendTheMessage(msg); // Uses UpstreamQ to send
            }
        }

        // Cycle to the next channel and delay 33ms
        current_idx = (current_idx + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void ADC_Setup(int idx) {
    if (!adc1_handle) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    }

    g_adc[idx].active = true;

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, get_adc_channel(g_adc[idx].iGPIO), &config));

    if (!adc_task_created) {
        xTaskCreate(adc_task, "adc_task", 3072, NULL, 5, NULL);
        adc_task_created = true;
    }
    
    ESP_LOGI(TAG, "ADC %s Initialized on GPIO %d", g_adc[idx].name, g_adc[idx].iGPIO);
}

int ADC_GetCurrentValue(int idx) {
    if (!g_adc[idx].active) return 0;
    int val = 0;
    adc_oneshot_read(adc1_handle, get_adc_channel(g_adc[idx].iGPIO), &val);
    return val;
}

// Threshold setters
void ADC_SetUpperThresh(int idx, int val) { g_adc[idx].UpperThresh = val; }
void ADC_SetLowerThresh(int idx, int val) { g_adc[idx].LowerThresh = val; }

// Helper to extract the 0-2 index from commands ending in A0, A1, A2
static int get_adc_idx(const char *cmd, const char *prefix) {
    if (strncmp(cmd, prefix, strlen(prefix)) == 0) {
        const char *suffix = cmd + strlen(prefix);
        if (strcmp(suffix, "A0") == 0) return 0;
        if (strcmp(suffix, "A1") == 0) return 1;
        if (strcmp(suffix, "A2") == 0) return 2;
    }
    return -1;
}

// --- Command Routing ---
bool adc_is_cmd(const char *cmd) {
    if (!cmd) return false;
    if (strcmp(cmd, "StreamADC") == 0) return true;
    if (get_adc_idx(cmd, "SetupADC") != -1) return true;
    if (get_adc_idx(cmd, "ReadADC") != -1) return true;
    if (get_adc_idx(cmd, "ADCupperThresh") != -1) return true;
    if (get_adc_idx(cmd, "ADCLowerThresh") != -1) return true;
    return false;
}

bool adc_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    int idx;
    char resp[32];

    if (strcmp(cmd, "StreamADC") == 0) {
        g_stream_serial = (value != 0);
        snprintf(resp, sizeof(resp), "ADC STREAM %s", g_stream_serial ? "ON" : "OFF");
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "SetupADC")) != -1) {
        ADC_Setup(idx);
        snprintf(resp, sizeof(resp), "SETUP ADC%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "ReadADC")) != -1) {
        snprintf(resp, sizeof(resp), "ADCval%s %d", g_adc[idx].name, ADC_GetCurrentValue(idx));
        mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "ADCupperThresh")) != -1) {
        ADC_SetUpperThresh(idx, value);
        snprintf(resp, sizeof(resp), "GOT ADCUPPERTHRESH%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "ADCLowerThresh")) != -1) {
        ADC_SetLowerThresh(idx, value);
        snprintf(resp, sizeof(resp), "GOT ADCLOWERTHRESH%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    }

    return false;
}