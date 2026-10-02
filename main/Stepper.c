// File: Stepper.c

#include "Stepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "mesh_zigbee.h"
#include "ledc_manager.h"

static const char *TAG = "STEPPER";

static int ap_pin = 18;
static int an_pin = 17;
static int bp_pin = 16;
static int bn_pin = 15;

// Speeds and Ramping
static int target_ms_per_step = 70;
static int max_ms_per_step = 200; // Starting/Ending speed for ramping
static int accel_steps = 0;
static int decel_steps = 0;

// Power
static int pwm_freq = 345;
static int pwm_duty_percent = 67; 
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
static bool stepper_active = false;

// --- Step Sequences (Scaled 0-255 for PWM duty calculations) ---

// Full Step
static const uint8_t full_step_seq[4][4] = {
    {255, 0, 255, 0},
    {0, 255, 255, 0},
    {0, 255, 0, 255},
    {255, 0, 0, 255}
};

// Half Step
static const uint8_t half_step_seq[8][4] = {
    {255, 0, 255, 0},
    {255, 0, 0,   0},
    {255, 0, 0, 255},
    {0,   0, 0, 255},
    {0, 255, 0, 255},
    {0, 255, 0,   0},
    {0, 255, 255, 0},
    {0,   0, 255, 0}
};

// 16-Microstep (Sine/Cosine approximation mapped to 0-255)
static const uint8_t micro_step_seq[16][4] = {
    {255, 0,   255, 0},
    {236, 0,   255, 0},
    {180, 0,   255, 0},
    {98,  0,   255, 0},
    {0,   0,   255, 0},
    {0,   98,  255, 0},
    {0,   180, 255, 0},
    {0,   236, 255, 0},
    {0,   255, 255, 0},
    {0,   255, 236, 0},
    {0,   255, 180, 0},
    {0,   255, 98,  0},
    {0,   255, 0,   0},
    {0,   255, 0,   98},
    {0,   255, 0,   180},
    {0,   255, 0,   236}
};

static const uint8_t (*current_sequence)[4] = full_step_seq;

static void configure_ledc_channel(int pin, ledc_channel_t channel) {
    if (pin < 0) return;
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

    // Scale the sequence's fractional value by the requested power level
    uint32_t d0 = (max_duty * current_sequence[step_idx][0]) / 255;
    uint32_t d1 = (max_duty * current_sequence[step_idx][1]) / 255;
    uint32_t d2 = (max_duty * current_sequence[step_idx][2]) / 255;
    uint32_t d3 = (max_duty * current_sequence[step_idx][3]) / 255;

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

void Stepper_Init(void) {
    if (stepper_active) return;

    if (!ledc_manager_alloc_timer(pwm_freq, &stepper_timer_num)) {
        ESP_LOGE(TAG, "Stepper Init Failed: No free timers");
        mesh_zigbee_send_text("ERR: NO FREE TIMERS");
        return;
    }

    if (!ledc_manager_alloc_channel(&ap_channel_num) || !ledc_manager_alloc_channel(&an_channel_num) ||
        !ledc_manager_alloc_channel(&bp_channel_num) || !ledc_manager_alloc_channel(&bn_channel_num)) {
        ESP_LOGE(TAG, "Stepper Init Failed: No free channels");
        mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
        return;
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

    configure_ledc_channel(ap_pin, ap_channel_num);
    configure_ledc_channel(an_pin, an_channel_num);
    configure_ledc_channel(bp_pin, bp_channel_num);
    configure_ledc_channel(bn_pin, bn_channel_num);

    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper Init. Freq: %dHz, Duty: %d%%", pwm_freq, pwm_duty_percent);
}

void Stepper_SetApGPIO(int pin) { ap_pin = pin; if (stepper_active) configure_ledc_channel(ap_pin, ap_channel_num); }
void Stepper_SetAnGPIO(int pin) { an_pin = pin; if (stepper_active) configure_ledc_channel(an_pin, an_channel_num); }
void Stepper_SetBpGPIO(int pin) { bp_pin = pin; if (stepper_active) configure_ledc_channel(bp_pin, bp_channel_num); }
void Stepper_SetBnGPIO(int pin) { bn_pin = pin; if (stepper_active) configure_ledc_channel(bn_pin, bn_channel_num); }

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

void Stepper_SetModeFullStep(void)  { current_sequence = full_step_seq;  sequence_length = 4;  }
void Stepper_SetModeHalfStep(void)  { current_sequence = half_step_seq;  sequence_length = 8;  }
void Stepper_SetModeMicroStep(void) { current_sequence = micro_step_seq; sequence_length = 16; }

// --- Command Routing ---
bool stepper_is_cmd(const char *cmd) {
    if (!cmd) return false;
    return (strcmp(cmd, "StepPWMduty") == 0 || strcmp(cmd, "HoldPWM") == 0 || 
            strcmp(cmd, "Accel") == 0 || strcmp(cmd, "Decel") == 0 || 
            strcmp(cmd, "FullStep") == 0 || strcmp(cmd, "HalfStep") == 0 || 
            strcmp(cmd, "MicroStep") == 0 || 
            // --- newly restored commands ---
            strcmp(cmd, "Step") == 0 || strcmp(cmd, "msPerStep") == 0 ||
            strcmp(cmd, "ApGPIO") == 0 || strcmp(cmd, "AnGPIO") == 0 ||
            strcmp(cmd, "BpGPIO") == 0 || strcmp(cmd, "BnGPIO") == 0 ||
            strcmp(cmd, "SetupStepper") == 0);
}

bool stepper_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (strcmp(cmd, "StepPWMduty") == 0) {
        Stepper_SetPWMDuty(value);
        if (!text_message) mesh_zigbee_send_text("GOT STEPPWMDUTY OK");
        return true;
    } else if (strcmp(cmd, "HoldPWM") == 0) {
        Stepper_SetHoldPWM(value);
        if (!text_message) mesh_zigbee_send_text("GOT HOLDPWM OK");
        return true;
    } else if (strcmp(cmd, "Accel") == 0) {
        Stepper_SetAccel(value);
        if (!text_message) mesh_zigbee_send_text("GOT ACCEL OK");
        return true;
    } else if (strcmp(cmd, "Decel") == 0) {
        Stepper_SetDecel(value);
        if (!text_message) mesh_zigbee_send_text("GOT DECEL OK");
        return true;
    } else if (strcmp(cmd, "FullStep") == 0) {
        Stepper_SetModeFullStep();
        if (!text_message) mesh_zigbee_send_text("GOT FULLSTEP OK");
        return true;
    } else if (strcmp(cmd, "HalfStep") == 0) {
        Stepper_SetModeHalfStep();
        if (!text_message) mesh_zigbee_send_text("GOT HALFSTEP OK");
        return true;
    } else if (strcmp(cmd, "MicroStep") == 0) {
        Stepper_SetModeMicroStep();
        if (!text_message) mesh_zigbee_send_text("GOT MICROSTEP OK");
        return true;
    } 
    // --- newly restored execution blocks ---
    else if (strcmp(cmd, "Step") == 0) {
        Stepper_Step(value);
        if (!text_message) mesh_zigbee_send_text("GOT STEP OK");
        return true;
    } else if (strcmp(cmd, "msPerStep") == 0) {
        Stepper_SetMsPerStep(value);
        if (!text_message) mesh_zigbee_send_text("GOT MSPERSTEP OK");
        return true;
    } else if (strcmp(cmd, "ApGPIO") == 0) {
        Stepper_SetApGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT APGPIO OK");
        return true;
    } else if (strcmp(cmd, "AnGPIO") == 0) {
        Stepper_SetAnGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT ANGPIO OK");
        return true;
    } else if (strcmp(cmd, "BpGPIO") == 0) {
        Stepper_SetBpGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT BPGPIO OK");
        return true;
    } else if (strcmp(cmd, "BnGPIO") == 0) {
        Stepper_SetBnGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT BNGPIO OK");
        return true;
    } else if (strcmp(cmd, "SetupStepper") == 0) {
        Stepper_Init();
        if (!text_message) mesh_zigbee_send_text("SETUP STEPPER OK");
        return true;
    }

    return false;
}