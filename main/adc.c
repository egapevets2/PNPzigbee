/*
 * adc.c - Analog-to-Digital Converter Subsystem & Threshold/Detent Controller
 *
 * Purpose:
 *   Manages ESP32-C6 ADC1 channels (A0/GPIO0, A1/GPIO1, A2/GPIO2) using the oneshot
 *   ADC driver. Supports two primary operational modes:
 *     1. Window Comparator Mode: Monitored with LowerThresh/UpperThresh hysteresis,
 *        reporting state changes ("ADC <chan> EXCEEDED UPPER", "ADC <chan> DROPPED BELOW LOWER").
 *     2. Potentiometer Detent Mode: Divides analog range into discrete percentage-based
 *        notches/detents (e.g. 10%) with hysteresis to prevent oscillation, reporting
 *        detent transitions ("ADC <chan> DETENT <N>"). Active range is calibrated across
 *        [50mV, 3.3V] for DC motor throttle control (ensuring positive 0% stop at full CCW
 *        and guaranteed 100% full throttle at full CW).
 *   Also supports one-shot reads and continuous serial streaming.
 *
 * Supported Commands:
 *   SetupADC [channel]
 *     - channel: 0 (A0), 1 (A1), or 2 (A2). Defaults to 1 (A1).
 *     Initializes the specified ADC channel in comparator mode with default thresholds.
 *     Aliases: SetupADCA0, SetupADCA1, SetupADCA2.
 *
 *   SetupADCpot [channel]
 *     - channel: 0 (A0), 1 (A1), or 2 (A2).
 *     Initializes or switches the specified channel into potentiometer detent mode.
 *     Aliases: ADCpot, SetupADCpotA0, SetupADCpotA1, SetupADCpotA2, ADCpotA0, ADCpotA1, ADCpotA2.
 *
 *   PotDetent [pct]
 *     - pct: Percentage per detent step (1 to 100, default 10).
 *     Configures potentiometer detent step size across active channels.
 *     Aliases: PotDetentA0, PotDetentA1, PotDetentA2 (for specific channel).
 *
 *   ReadADC [channel]
 *     - channel: 0 (A0), 1 (A1), or 2 (A2). Defaults to 1 (A1).
 *     Performs a settled multi-sample reading and replies "ADCval<name> <raw>" (0..4095).
 *     Aliases: ReadADCA0, ReadADCA1, ReadADCA2.
 *
 *   ADCupperThreshA0 <val> / ADCupperThreshA1 <val> / ADCupperThreshA2 <val>
 *     - val: Raw ADC count threshold (0..4095).
 *     Sets upper comparator threshold and engages comparator mode.
 *
 *   ADCLowerThreshA0 <val> / ADCLowerThreshA1 <val> / ADCLowerThreshA2 <val>
 *     - val: Raw ADC count threshold (0..4095).
 *     Sets lower comparator threshold and engages comparator mode.
 *
 *   StreamADC <0|1>
 *     - 1 to enable, 0 to disable continuous streaming of raw ADC values to local serial port.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "UpstreamQ.h"
#include "adc.h"
#include "mesh_zigbee.h"
#include "mesh_serial.h"

static const char *TAG = "ADC_CTRL";

/*
 * ADC Hardware & Potentiometer Calibration Scaling:
 *
 * 1. Hardware Calibration:
 *    - ESP32-C6 ADC1 with ADC_ATTEN_DB_12 has a nominal saturation voltage of ~3.96V (3960mV)
 *      across the 12-bit (0..4095) SAR conversion range.
 *
 * 2. High-Side Scaling Rationale (3.3V Rail Alignment):
 *    - Potentiometers in ESP32 embedded designs are powered from the 3.3V VDD rail to remain
 *      within the 3.6V absolute maximum GPIO rating without requiring external divider networks.
 *    - If full-scale were left at the 3.96V hardware clipping limit, a 3.3V potentiometer at full
 *      clockwise (CW) rotation would max out at ~3412 raw counts (only ~83% of span). This would
 *      prevent brushed DC motors (e.g. Cytron MD10C) from ever reaching 100% PWM power (topping out
 *      at detent 16 of 20).
 *    - Aligning ADC_FULL_SCALE_MV to 3300.0f (~3412.5 counts) scales the active potentiometer span
 *      so that full CW rotation reaches 100% full scale (pos = 1.0 / detent N).
 *
 * 3. Low-Side Scaling Rationale (50mV Floor & Deadband):
 *    - Wiper Contact Resistance: Real potentiometers rarely achieve true 0.00 ohms at full counter-
 *      clockwise (CCW) rotation; wiper contact, track ends, and lead resistance retain 1-20 ohms.
 *    - Motor Driver Ground Bounce: Heavy DC motor return currents flowing through shared GND paths
 *      generate I*R ground voltage shifts (typically 10-40mV), lifting the apparent wiper voltage.
 *    - SAR ADC Low-End Non-Linearity: ESP32-C6 SAR ADC1 at 12dB attenuation exhibits non-linearity
 *      below ~60-80mV and rarely reads true 0 raw counts.
 *    - Hysteresis Deadband Trapping: Step hysteresis (0.15 margin) requires dropping below 0.35 steps
 *      to transition down to detent 0. Without an explicit zero floor, small ground shifts trap the
 *      state machine at detent 1 or 2, preventing positive motor shut-off.
 *    - Tactile Deadband: 50mV corresponds to ~1.5% of 3.3V, or ~4 degrees of rotation on a 270-degree
 *      pot, providing an ideal natural tactile deadband that guarantees 0% motor PWM at full CCW stop.
 */
#define ADC_HW_FULL_SCALE_COUNTS 4095.0f // 12-bit ADC hardware count
#define ADC_HW_FULL_SCALE_MV     3960.0f // ESP32-C6 ADC 12dB attenuation saturation voltage (~3.96V)
#define ADC_FULL_SCALE_MV        3300.0f // Potentiometer active full scale matched to 3.3V rail
#define ADC_FULL_SCALE_COUNTS    ((ADC_FULL_SCALE_MV / ADC_HW_FULL_SCALE_MV) * ADC_HW_FULL_SCALE_COUNTS) // ~3412.5 counts (~3.30V)
#define ADC_ZERO_OFFSET_MV       50.0f   // 50mV zero threshold offset (positive shut-off deadband)
#define ADC_ZERO_OFFSET_COUNTS   ((ADC_ZERO_OFFSET_MV / ADC_HW_FULL_SCALE_MV) * ADC_HW_FULL_SCALE_COUNTS) // ~51.7 counts (~50mV)
#define POT_HYST_MARGIN          0.15f   // 0.50 (halfway) + 0.15 = 0.65 (65% of a step to snap into next detent)

typedef enum {
    ADC_MODE_COMPARATOR = 0,
    ADC_MODE_POT        = 1
} adc_mode_t;

// Define the struct based on requirements
typedef struct {
    bool active;
    int iGPIO;
    int LowerThresh;
    int UpperThresh;
    bool Comparator;
    adc_mode_t mode;
    int PotDetentPct;
    int CurrentDetent;
    int ReportedDetent;
    const char* name;
} adc_config_t;

static adc_config_t g_adc[3] = {
    { .active = false, .iGPIO = 0, .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .mode = ADC_MODE_COMPARATOR, .PotDetentPct = 10, .CurrentDetent = -1, .ReportedDetent = -1, .name = "A0" },
    { .active = false, .iGPIO = 1, .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .mode = ADC_MODE_COMPARATOR, .PotDetentPct = 10, .CurrentDetent = -1, .ReportedDetent = -1, .name = "A1" },
    { .active = false, .iGPIO = 2, .LowerThresh = 111, .UpperThresh = 334, .Comparator = false, .mode = ADC_MODE_COMPARATOR, .PotDetentPct = 10, .CurrentDetent = -1, .ReportedDetent = -1, .name = "A2" }
};

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static SemaphoreHandle_t s_adc_mutex = NULL;
static bool adc_task_created = false;
static bool g_stream_serial = false;

static void ensure_adc_mutex(void) {
    if (!s_adc_mutex) {
        s_adc_mutex = xSemaphoreCreateMutex();
    }
}

// Helper to map GPIO to ESP32-C6 ADC1 Channels
static adc_channel_t get_adc_channel(int gpio) {
    switch (gpio) {
        case 0:  return ADC_CHANNEL_0;
        case 1:  return ADC_CHANNEL_1;
        case 2:  return ADC_CHANNEL_2;
        case 3:  return ADC_CHANNEL_3;
        case 4:  return ADC_CHANNEL_4;
        case 5:  return ADC_CHANNEL_5;
        case 6:  return ADC_CHANNEL_6;
        default: return ADC_CHANNEL_1;
    }
}

// Read channel with 1 dummy sample (for SAR mux settling across resistor dividers) + 4x average
static int read_adc_channel_settled(adc_channel_t chan) {
    int raw = 0;
    int sum = 0;
    adc_oneshot_read(adc1_handle, chan, &raw); // Dummy read to settle S&H capacitor
    for (int i = 0; i < 4; i++) {
        if (adc_oneshot_read(adc1_handle, chan, &raw) == ESP_OK) {
            sum += raw;
        }
    }
    return sum / 4;
}

static float compute_pot_position(int raw_val, int pct) {
    if (pct < 1) pct = 1;
    float step_frac = (float)pct / 100.0f;

    // Floor to 0.0f whenever input is at or below 50mV zero offset
    // Guarantees that a full CCW potentiometer produces completely stopped 0% PWM
    if ((float)raw_val <= ADC_ZERO_OFFSET_COUNTS) {
        return 0.0f;
    }

    // Shift the active linear range from [50mV, 3.3V] to [0.0, 1.0]
    float effective_span = ADC_FULL_SCALE_COUNTS - ADC_ZERO_OFFSET_COUNTS;
    float frac = ((float)raw_val - ADC_ZERO_OFFSET_COUNTS) / effective_span;
    if (frac > 1.0f) frac = 1.0f;

    return frac / step_frac;
}

// The task that implements the comparator or ADCpot detent with hysteresis
static void adc_task(void *arg) {
    int current_idx = 0;
    while (1) {
        if (g_adc[current_idx].active) {
            int raw_val = 0;
            ensure_adc_mutex();
            if (xSemaphoreTake(s_adc_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                raw_val = read_adc_channel_settled(get_adc_channel(g_adc[current_idx].iGPIO));
                xSemaphoreGive(s_adc_mutex);
            }

            if (g_stream_serial) {
                mesh_serial_writef("ADC%s: %d\r\n", g_adc[current_idx].name, raw_val);
            }

            if (g_adc[current_idx].mode == ADC_MODE_POT) {
                float pos = compute_pot_position(raw_val, g_adc[current_idx].PotDetentPct);
                if (g_adc[current_idx].CurrentDetent < 0) {
                    int init_detent = (int)(pos + 0.5f);
                    g_adc[current_idx].CurrentDetent = init_detent;
                    g_adc[current_idx].ReportedDetent = init_detent;
                } else {
                    int cur = g_adc[current_idx].CurrentDetent;
                    int new_detent = cur;
                    float diff = pos - (float)cur;
                    const float snap_thresh = 0.5f + POT_HYST_MARGIN; // 0.65 (65% of a step)

                    if (diff >= snap_thresh) {
                        new_detent = (int)floorf(pos - POT_HYST_MARGIN + 0.5f);
                    } else if (diff <= -snap_thresh) {
                        new_detent = (int)ceilf(pos + POT_HYST_MARGIN - 0.5f);
                        if (new_detent < 0) new_detent = 0;
                    }

                    // Unconditional floor: if position is 0.0f (<= 50mV), force detent 0
                    if (pos <= 0.0f) {
                        new_detent = 0;
                    }

                    if (new_detent != cur) {
                        g_adc[current_idx].CurrentDetent = new_detent;
                        ESP_LOGI(TAG, "ADCpot %s snapped to detent %d (raw=%d, pos=%.2f)",
                                 g_adc[current_idx].name, new_detent, raw_val, pos);
                    }

                    // Guarantee that the latest/final resting detent is never dropped:
                    // keep attempting to enqueue until ReportedDetent catches up to CurrentDetent.
                    if (g_adc[current_idx].CurrentDetent != g_adc[current_idx].ReportedDetent) {
                        char msg[32];
                        snprintf(msg, sizeof(msg), "ADCpot%s %d",
                                 g_adc[current_idx].name, g_adc[current_idx].CurrentDetent);
                        if (SendTheMessage(msg)) {
                            g_adc[current_idx].ReportedDetent = g_adc[current_idx].CurrentDetent;
                        }
                    }
                }
            } else {
                bool old_state = g_adc[current_idx].Comparator;

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
        }

        // Cycle to the next channel and delay 33ms
        current_idx = (current_idx + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

static void adc_hw_init_channel(int idx) {
    if (idx < 0 || idx >= 3) return;
    ensure_adc_mutex();

    xSemaphoreTake(s_adc_mutex, portMAX_DELAY);

    if (!adc1_handle) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    }

    // Release any prior digital/PWM pin drivers and disable internal pull-ups/pull-downs
    // so external resistor dividers on A0/A1/A2 see a high-impedance analog input.
    int gpio_num = g_adc[idx].iGPIO;
    gpio_reset_pin(gpio_num);
    gpio_pullup_dis(gpio_num);
    gpio_pulldown_dis(gpio_num);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, get_adc_channel(gpio_num), &config));

    g_adc[idx].active = true;

    xSemaphoreGive(s_adc_mutex);

    if (!adc_task_created) {
        xTaskCreate(adc_task, "adc_task", 3072, NULL, 5, NULL);
        adc_task_created = true;
    }
}

void ADC_Setup(int idx) {
    if (idx < 0 || idx >= 3) return;
    adc_hw_init_channel(idx);
    g_adc[idx].mode = ADC_MODE_COMPARATOR;
    ESP_LOGI(TAG, "ADC %s Initialized on GPIO %d (Comparator mode)", g_adc[idx].name, g_adc[idx].iGPIO);
}

void ADC_SetupPot(int idx) {
    if (idx < 0 || idx >= 3) return;
    adc_hw_init_channel(idx);
    int raw = ADC_GetCurrentValue(idx);
    float pos = compute_pot_position(raw, g_adc[idx].PotDetentPct);
    int init_detent = (int)(pos + 0.5f);
    g_adc[idx].CurrentDetent = init_detent;
    g_adc[idx].ReportedDetent = init_detent;
    g_adc[idx].mode = ADC_MODE_POT;
    ESP_LOGI(TAG, "ADC %s Initialized on GPIO %d (ADCpot mode, Detent=%d%%, InitialDetent=%d)",
             g_adc[idx].name, g_adc[idx].iGPIO, g_adc[idx].PotDetentPct, g_adc[idx].CurrentDetent);
}

int ADC_GetCurrentValue(int idx) {
    if (idx < 0 || idx >= 3) return 0;
    if (!g_adc[idx].active) {
        ADC_Setup(idx);
    }
    int val = 0;
    ensure_adc_mutex();
    if (xSemaphoreTake(s_adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        val = read_adc_channel_settled(get_adc_channel(g_adc[idx].iGPIO));
        xSemaphoreGive(s_adc_mutex);
    }
    return val;
}

// Threshold and PotDetent setters
void ADC_SetUpperThresh(int idx, int val) {
    if (idx >= 0 && idx < 3) {
        g_adc[idx].UpperThresh = val;
        g_adc[idx].mode = ADC_MODE_COMPARATOR;
    }
}

void ADC_SetLowerThresh(int idx, int val) {
    if (idx >= 0 && idx < 3) {
        g_adc[idx].LowerThresh = val;
        g_adc[idx].mode = ADC_MODE_COMPARATOR;
    }
}

void ADC_SetPotDetent(int idx, int pct) {
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    if (idx >= 0 && idx < 3) {
        g_adc[idx].PotDetentPct = pct;
        if (g_adc[idx].active) {
            int raw = ADC_GetCurrentValue(idx);
            float pos = compute_pot_position(raw, pct);
            int init_detent = (int)(pos + 0.5f);
            g_adc[idx].CurrentDetent = init_detent;
            g_adc[idx].ReportedDetent = init_detent;
        } else {
            g_adc[idx].CurrentDetent = -1;
            g_adc[idx].ReportedDetent = -1;
        }
    }
}

// Helper to extract the 0-2 index from commands ending in A0/A1/A2 or 0/1/2
static int get_adc_idx(const char *cmd, const char *prefix) {
    if (strncasecmp(cmd, prefix, strlen(prefix)) == 0) {
        const char *suffix = cmd + strlen(prefix);
        if (strcasecmp(suffix, "A0") == 0 || strcmp(suffix, "0") == 0) return 0;
        if (strcasecmp(suffix, "A1") == 0 || strcmp(suffix, "1") == 0) return 1;
        if (strcasecmp(suffix, "A2") == 0 || strcmp(suffix, "2") == 0) return 2;
    }
    return -1;
}

// --- Command Routing ---
bool adc_is_cmd(const char *cmd) {
    if (!cmd) return false;
    if (strcasecmp(cmd, "StreamADC") == 0) return true;
    if (strcasecmp(cmd, "SetupADC") == 0 || get_adc_idx(cmd, "SetupADC") != -1) return true;
    if (strcasecmp(cmd, "ADCpot") == 0 || get_adc_idx(cmd, "ADCpot") != -1) return true;
    if (strcasecmp(cmd, "SetupADCpot") == 0 || get_adc_idx(cmd, "SetupADCpot") != -1) return true;
    if (strcasecmp(cmd, "PotDetent") == 0 || get_adc_idx(cmd, "PotDetent") != -1) return true;
    if (strcasecmp(cmd, "ReadADC") == 0  || get_adc_idx(cmd, "ReadADC") != -1) return true;
    if (get_adc_idx(cmd, "ADCupperThresh") != -1) return true;
    if (get_adc_idx(cmd, "ADCLowerThresh") != -1) return true;
    return false;
}

bool adc_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    int idx;
    char resp[32];

    if (strcasecmp(cmd, "StreamADC") == 0) {
        g_stream_serial = (value != 0);
        snprintf(resp, sizeof(resp), "ADC STREAM %s", g_stream_serial ? "ON" : "OFF");
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "SetupADC") == 0) {
        idx = (value >= 0 && value <= 2) ? value : 1;
        ADC_Setup(idx);
        snprintf(resp, sizeof(resp), "SETUP ADC%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "SetupADC")) != -1) {
        ADC_Setup(idx);
        snprintf(resp, sizeof(resp), "SETUP ADC%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "ADCpot")) != -1 || (idx = get_adc_idx(cmd, "SetupADCpot")) != -1) {
        ADC_SetupPot(idx);
        snprintf(resp, sizeof(resp), "SETUP ADCPOT%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "ADCpot") == 0 || strcasecmp(cmd, "SetupADCpot") == 0) {
        if (value >= 1 && value <= 2) {
            ADC_SetupPot(value);
            snprintf(resp, sizeof(resp), "SETUP ADCPOT%s OK", g_adc[value].name);
        } else {
            // Check if any channel is already active; if so, switch active channel(s) to ADCpot mode.
            // Otherwise initialize A1 (default pot channel) or A0 if value == 0.
            bool any_active = false;
            int last_idx = 1;
            for (int i = 0; i < 3; i++) {
                if (g_adc[i].active) {
                    ADC_SetupPot(i);
                    any_active = true;
                    last_idx = i;
                }
            }
            if (!any_active) {
                last_idx = (value >= 0 && value <= 2) ? value : 1;
                ADC_SetupPot(last_idx);
            }
            snprintf(resp, sizeof(resp), "SETUP ADCPOT%s OK", g_adc[last_idx].name);
        }
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if ((idx = get_adc_idx(cmd, "PotDetent")) != -1) {
        int pct = (value > 0) ? value : 10;
        ADC_SetPotDetent(idx, pct);
        snprintf(resp, sizeof(resp), "GOT POTDETENT%s OK", g_adc[idx].name);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "PotDetent") == 0) {
        int pct = (value > 0) ? value : 10;
        for (int i = 0; i < 3; i++) {
            ADC_SetPotDetent(i, pct);
        }
        snprintf(resp, sizeof(resp), "GOT POTDETENT %d OK", pct);
        if (!text_message) mesh_zigbee_send_text(resp);
        return true;
    } else if (strcasecmp(cmd, "ReadADC") == 0) {
        idx = (value >= 0 && value <= 2) ? value : 1;
        snprintf(resp, sizeof(resp), "ADCval%s %d", g_adc[idx].name, ADC_GetCurrentValue(idx));
        mesh_zigbee_send_text(resp);
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
