/*
 * Stepper.c - 4-Wire Bipolar Stepper Motor Driver Subsystem
 *
 * Purpose:
 *   Drives 4-wire bipolar stepper motors (A+, A-, B+, B- coils; default pins: Ap=1, An=21, Bp=2, Bn=0)
 *   using 8-bit ESP32-C6 LEDC PWM channels allocated through ledc_manager at 25 kHz ultrasonic default.
 *   Automatically calls UnipolarStep_Deinit() on initialization to prevent pin/LEDC conflicts.
 *   Supports full-step (4-step) and half-step (8-step) sequences, programmable step timing (msPerStep),
 *   active duty cycle current limiting (StepPWMduty), holding current (HoldPWM), and trapezoidal
 *   acceleration/deceleration step ramping. Commands accept space-delimited or '=' syntax.
 *
 * Supported Commands:
 *   SetupStepper
 *     Allocates LEDC timer and 4 PWM channels for coils and launches stepper background task.
 *     Replies "SETUP STEPPER OK".
 *
 *   Step <steps>
 *     - steps: Relative step count (positive = forward, negative = reverse).
 *     Commands stepper movement with acceleration and deceleration profiling ("GOT STEP OK").
 *
 *   msPerStep <ms>
 *     - ms: Target step period in milliseconds (default 20, "GOT MSPERSTEP OK").
 *
 *   StepPWMduty <percent>
 *     - percent: Active coil PWM duty cycle percentage (0..100, default 25, "GOT STEPPWMDUTY OK").
 *
 *   StepPWMfreq <freq>
 *     - freq: PWM switching frequency in Hz (default 25000, "GOT STEPPWMFREQ OK").
 *
 *   HoldPWM <percent>
 *     - percent: Stationary holding duty cycle percentage (0..100, default 0, "GOT HOLDPWM OK").
 *
 *   Accel <steps> / Decel <steps>
 *     - steps: Number of acceleration / deceleration ramp steps between 200ms and target msPerStep
 *       ("GOT ACCEL OK" / "GOT DECEL OK").
 *
 *   FullStep / HalfStep
 *     Configures 4-step full-step ("GOT FULLSTEP OK") or 8-step half-step ("GOT HALFSTEP OK") mode.
 *
 *   ApGPIO <pin> / AnGPIO <pin> / BpGPIO <pin> / BnGPIO <pin>
 *     - pin: GPIO pin number for coil A+, A-, B+, or B- (defaults: 1, 21, 2, 0; "GOT <AP|AN|BP|BN>GPIO OK").
 */

#include "Stepper.h"
#include "unipolarStep.h"
#include "pwm_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "mesh_zigbee.h"
#include "ledc_manager.h"

static const char *TAG = "STEPPER";

static int ap_pin = 1;
static int an_pin = 21;
static int bp_pin = 2;
static int bn_pin = 0;

// Speeds and Ramping
static int target_ms_per_step = 20;
static int max_ms_per_step = 200; // Starting/Ending speed for ramping
static int accel_steps = 0;
static int decel_steps = 0;

// Power
static int pwm_freq = 25000;
static int pwm_duty_percent = 25; 
static int hold_pwm_percent = 0; 

// State
static volatile int target_steps = 0;
static int steps_taken = 0;
static int steps_remaining = 0;
static int current_step_index = 0;
static int sequence_length = 4;

static ledc_timer_t stepper_timer_num;
static ledc_channel_t ap_channel_num;
static ledc_channel_t an_channel_num;
static ledc_channel_t bp_channel_num;
static ledc_channel_t bn_channel_num;
static int s_timer_alloc = -1;
static int s_ch_alloc[4] = {-1, -1, -1, -1};
static bool stepper_active = false;
static bool channels_allocated = false;
static TaskHandle_t s_stepper_task_handle = NULL;

// --- Step Sequences (1 = coil active at StepPWMduty, 0 = off) ---

// Full Step
static const uint8_t full_step_seq[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}
};

// Half Step
static const uint8_t half_step_seq[8][4] = {
    {1, 0, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
    {1, 0, 0, 0}
};

static const uint8_t (*current_sequence)[4] = full_step_seq;

static void configure_ledc_channel(int pin, ledc_channel_t channel) {
    if (pin < 0) return;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    ledc_channel_config_t ledc_channel_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = channel,
        .timer_sel      = stepper_timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = pin,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_cfg);
}

// Applies the step pattern using a specific overall duty percentage
static void apply_step(int step_idx, int duty_percent) {
    if (!stepper_active) return;

    uint32_t max_duty = (duty_percent * 255) / 100; 

    uint32_t d0 = current_sequence[step_idx][0] ? max_duty : 0;
    uint32_t d1 = current_sequence[step_idx][1] ? max_duty : 0;
    uint32_t d2 = current_sequence[step_idx][2] ? max_duty : 0;
    uint32_t d3 = current_sequence[step_idx][3] ? max_duty : 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, ap_channel_num, d0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, an_channel_num, d1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, bp_channel_num, d2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, bn_channel_num, d3);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, ap_channel_num);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, an_channel_num);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, bp_channel_num);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, bn_channel_num);
}

// Calculates dynamic delay for Accel/Decel
static int calculate_current_delay() {
    if (accel_steps > 0 && steps_taken < accel_steps) {
        // Accelerating (decreasing delay)
        float progress = (float)steps_taken / accel_steps;
        return max_ms_per_step - (progress * (max_ms_per_step - target_ms_per_step));
    } 
    if (decel_steps > 0 && steps_remaining <= decel_steps) {
        // Decelerating (increasing delay)
        float progress = 1.0 - ((float)steps_remaining / decel_steps);
        return target_ms_per_step + (progress * (max_ms_per_step - target_ms_per_step));
    }
    return target_ms_per_step; // Cruising
}

static void stepper_task(void *arg) {
    while (1) {
        if (target_steps != 0) {
            // Setup ramping counters if starting a new move
            if (steps_remaining == 0) {
                steps_taken = 0;
                steps_remaining = abs(target_steps);
            }

            if (target_steps > 0) {
                current_step_index = (current_step_index + 1) % sequence_length;
                target_steps--;
            } else if (target_steps < 0) {
                current_step_index = (current_step_index - 1 + sequence_length) % sequence_length;
                target_steps++;
            }
            
            steps_taken++;
            steps_remaining--;

            apply_step(current_step_index, pwm_duty_percent);
            vTaskDelay(pdMS_TO_TICKS(calculate_current_delay()));

        } else {
            // Idle state: Apply holding torque or freewheel
            steps_taken = 0;
            steps_remaining = 0;
            
            if (hold_pwm_percent > 0) {
                apply_step(current_step_index, hold_pwm_percent);
            } else {
                apply_step(current_step_index, 0); // Freewheel
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static int bound_pins[4] = {-1, -1, -1, -1};

static void reset_pin_clean(int pin) {
    if (pin < 0) return;
    gpio_reset_pin(pin);
    gpio_pullup_dis(pin);
    gpio_pulldown_dis(pin);
}

static void rebind_all_stepper_channels(void) {
    for (int i = 0; i < 4; i++) {
        if (bound_pins[i] >= 0) {
            reset_pin_clean(bound_pins[i]);
        }
    }
    reset_pin_clean(ap_pin);
    reset_pin_clean(an_pin);
    reset_pin_clean(bp_pin);
    reset_pin_clean(bn_pin);

    configure_ledc_channel(ap_pin, ap_channel_num);
    configure_ledc_channel(an_pin, an_channel_num);
    configure_ledc_channel(bp_pin, bp_channel_num);
    configure_ledc_channel(bn_pin, bn_channel_num);

    bound_pins[0] = ap_pin;
    bound_pins[1] = an_pin;
    bound_pins[2] = bp_pin;
    bound_pins[3] = bn_pin;
}

void Stepper_Deinit(void) {
    bool any_alloc = (s_timer_alloc >= 0) ||
                     (s_ch_alloc[0] >= 0) || (s_ch_alloc[1] >= 0) ||
                     (s_ch_alloc[2] >= 0) || (s_ch_alloc[3] >= 0);
    if (!stepper_active && s_stepper_task_handle == NULL && !channels_allocated && !any_alloc) {
        return;
    }

    // Mute all duty outputs while still active
    if (stepper_active) {
        apply_step(0, 0);
    }

    stepper_active = false;
    target_steps = 0;
    steps_taken = 0;
    steps_remaining = 0;

    if (s_stepper_task_handle != NULL) {
        TaskHandle_t h = s_stepper_task_handle;
        s_stepper_task_handle = NULL;
        vTaskDelete(h);
    }

    // Stop and free LEDC channels and timer back to ledc_manager
    for (int i = 0; i < 4; i++) {
        if (s_ch_alloc[i] >= 0) {
            ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_ch_alloc[i], 0);
            ledc_manager_free_channel((ledc_channel_t)s_ch_alloc[i]);
            s_ch_alloc[i] = -1;
        }
    }
    if (s_timer_alloc >= 0) {
        ledc_manager_free_timer((ledc_timer_t)s_timer_alloc);
        s_timer_alloc = -1;
    }
    channels_allocated = false;

    // Reset pins and release from bound list
    for (int i = 0; i < 4; i++) {
        if (bound_pins[i] >= 0) {
            reset_pin_clean(bound_pins[i]);
            bound_pins[i] = -1;
        }
    }
    reset_pin_clean(ap_pin);
    reset_pin_clean(an_pin);
    reset_pin_clean(bp_pin);
    reset_pin_clean(bn_pin);

    ESP_LOGI(TAG, "Bipolar Stepper deinitialized");
}

void Stepper_Init(void) {
    // Cleanly deactivate unipolar stepper and general-purpose PWM channels to prevent pin/LEDC exhaustion
    UnipolarStep_Deinit();
    pwm_driver_deinit_all();

    if (!channels_allocated) {
        if (!ledc_manager_alloc_timer(pwm_freq, &stepper_timer_num)) {
            ESP_LOGE(TAG, "Stepper Init Failed: No free timers");
            mesh_zigbee_send_text("ERR: NO FREE TIMERS");
            return;
        }
        s_timer_alloc = (int)stepper_timer_num;

        ledc_channel_t *ch_ptrs[4] = { &ap_channel_num, &an_channel_num, &bp_channel_num, &bn_channel_num };
        for (int i = 0; i < 4; i++) {
            if (!ledc_manager_alloc_channel(ch_ptrs[i])) {
                ESP_LOGE(TAG, "Stepper Init Failed: No free channels");
                Stepper_Deinit();
                mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
                return;
            }
            s_ch_alloc[i] = (int)(*ch_ptrs[i]);
        }
        channels_allocated = true;
    }

    stepper_active = true;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = stepper_timer_num,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = pwm_freq, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    rebind_all_stepper_channels();

    if (s_stepper_task_handle == NULL) {
        xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, &s_stepper_task_handle);
    }
    ESP_LOGI(TAG, "Stepper Init. Pins: Ap=%d, An=%d, Bp=%d, Bn=%d | Freq: %dHz, Duty: %d%%",
             ap_pin, an_pin, bp_pin, bn_pin, pwm_freq, pwm_duty_percent);
}

void Stepper_SetApGPIO(int pin) { ap_pin = pin; if (stepper_active) rebind_all_stepper_channels(); }
void Stepper_SetAnGPIO(int pin) { an_pin = pin; if (stepper_active) rebind_all_stepper_channels(); }
void Stepper_SetBpGPIO(int pin) { bp_pin = pin; if (stepper_active) rebind_all_stepper_channels(); }
void Stepper_SetBnGPIO(int pin) { bn_pin = pin; if (stepper_active) rebind_all_stepper_channels(); }

void Stepper_SetMsPerStep(int ms) { target_ms_per_step = ms > 0 ? ms : 1; }
void Stepper_Step(int steps) { target_steps += steps; }

void Stepper_SetPWMFreq(int freq) {
    pwm_freq = freq;
    if (stepper_active) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, stepper_timer_num, pwm_freq);
    }
}
void Stepper_SetPWMDuty(int duty_percent) {
    pwm_duty_percent = duty_percent < 0 ? 0 : (duty_percent > 100 ? 100 : duty_percent);
}

void Stepper_SetHoldPWM(int duty_percent) {
    hold_pwm_percent = duty_percent < 0 ? 0 : (duty_percent > 100 ? 100 : duty_percent);
}

void Stepper_SetAccel(int steps) { accel_steps = steps >= 0 ? steps : 0; }
void Stepper_SetDecel(int steps) { decel_steps = steps >= 0 ? steps : 0; }

void Stepper_SetModeFullStep(void)  { current_step_index = 0; current_sequence = full_step_seq;  sequence_length = 4;  }
void Stepper_SetModeHalfStep(void)  { current_step_index = 0; current_sequence = half_step_seq;  sequence_length = 8;  }

static void sanitize_stepper_cmd(const char *src, char *dst, size_t max_len, int *has_eq_val, int16_t *eq_val)
{
    if (has_eq_val) *has_eq_val = 0;
    size_t i = 0;
    while (src[i] != '\0' && src[i] != '=' && i < max_len - 1) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
    if (src[i] == '=' && has_eq_val && eq_val) {
        *has_eq_val = 1;
        *eq_val = (int16_t)atoi(&src[i + 1]);
    }
}

// --- Command Routing ---
bool stepper_is_cmd(const char *cmd) {
    if (!cmd) return false;
    char name[32];
    sanitize_stepper_cmd(cmd, name, sizeof(name), NULL, NULL);
    return (strcmp(name, "StepPWMduty") == 0 || strcmp(name, "StepPWMfreq") == 0 ||
            strcmp(name, "HoldPWM") == 0 || 
            strcmp(name, "Accel") == 0 || strcmp(name, "Decel") == 0 || 
            strcmp(name, "FullStep") == 0 || strcmp(name, "HalfStep") == 0 || 
            // --- newly restored commands ---
            strcmp(name, "Step") == 0 || strcmp(name, "msPerStep") == 0 ||
            strcmp(name, "ApGPIO") == 0 || strcmp(name, "AnGPIO") == 0 ||
            strcmp(name, "BpGPIO") == 0 || strcmp(name, "BnGPIO") == 0 ||
            strcmp(name, "SetupStepper") == 0);
}

bool stepper_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (!cmd) return false;
    char name[32];
    int has_eq_val = 0;
    int16_t eq_val = 0;
    sanitize_stepper_cmd(cmd, name, sizeof(name), &has_eq_val, &eq_val);
    if (has_eq_val) {
        value = eq_val;
    }

    if (strcmp(name, "StepPWMduty") == 0) {
        Stepper_SetPWMDuty(value);
        if (!text_message) mesh_zigbee_send_text("GOT STEPPWMDUTY OK");
        return true;
    } else if (strcmp(name, "StepPWMfreq") == 0) {
        Stepper_SetPWMFreq((uint16_t)value);
        if (!text_message) mesh_zigbee_send_text("GOT STEPPWMFREQ OK");
        return true;
    } else if (strcmp(name, "HoldPWM") == 0) {
        Stepper_SetHoldPWM(value);
        if (!text_message) mesh_zigbee_send_text("GOT HOLDPWM OK");
        return true;
    } else if (strcmp(name, "Accel") == 0) {
        Stepper_SetAccel(value);
        if (!text_message) mesh_zigbee_send_text("GOT ACCEL OK");
        return true;
    } else if (strcmp(name, "Decel") == 0) {
        Stepper_SetDecel(value);
        if (!text_message) mesh_zigbee_send_text("GOT DECEL OK");
        return true;
    } else if (strcmp(name, "FullStep") == 0) {
        Stepper_SetModeFullStep();
        if (!text_message) mesh_zigbee_send_text("GOT FULLSTEP OK");
        return true;
    } else if (strcmp(name, "HalfStep") == 0) {
        Stepper_SetModeHalfStep();
        if (!text_message) mesh_zigbee_send_text("GOT HALFSTEP OK");
        return true;
    } 
    // --- newly restored execution blocks ---
    else if (strcmp(name, "Step") == 0) {
        Stepper_Step(value);
        if (!text_message) mesh_zigbee_send_text("GOT STEP OK");
        return true;
    } else if (strcmp(name, "msPerStep") == 0) {
        Stepper_SetMsPerStep(value);
        if (!text_message) mesh_zigbee_send_text("GOT MSPERSTEP OK");
        return true;
    } else if (strcmp(name, "ApGPIO") == 0) {
        Stepper_SetApGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT APGPIO OK");
        return true;
    } else if (strcmp(name, "AnGPIO") == 0) {
        Stepper_SetAnGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT ANGPIO OK");
        return true;
    } else if (strcmp(name, "BpGPIO") == 0) {
        Stepper_SetBpGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT BPGPIO OK");
        return true;
    } else if (strcmp(name, "BnGPIO") == 0) {
        Stepper_SetBnGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT BNGPIO OK");
        return true;
    } else if (strcmp(name, "SetupStepper") == 0) {
        Stepper_Init();
        if (!text_message) mesh_zigbee_send_text("SETUP STEPPER OK");
        return true;
    }

    return false;
}