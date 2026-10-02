// File: pwm_driver.c

#include "pwm_driver.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#include "mesh_zigbee.h" 
#include "ledc_manager.h" // <-- Include the new central resource manager

static const char *TAG = "PWM_DRIVER";

typedef struct {
    bool active;           // Tracks if SetupPWMx has been called and resources allocated
    int current_gpio;
    int current_freq;
    int pwm_slew_rate; 
    int target_pwm;
    float current_pwm;
    ledc_timer_t timer_num;
    ledc_channel_t channel_num;
} pwm_channel_state_t;

// State array for 4 independent channels (active defaults to false)
// Index 0 maps to "pwm1", index 3 maps to "pwm4"
// Timer and Channel numbers are left uninitialized because the manager provides them
static pwm_channel_state_t pwm_states[NUM_PWM_CHANNELS] = {
    { .active = false, .current_gpio = GPIO_NUM_2, .current_freq = 50, .pwm_slew_rate = 0, .target_pwm = 77, .current_pwm = 77.0f },
    { .active = false, .current_gpio = GPIO_NUM_3, .current_freq = 50, .pwm_slew_rate = 0, .target_pwm = 0,  .current_pwm = 0.0f  },
    { .active = false, .current_gpio = GPIO_NUM_4, .current_freq = 50, .pwm_slew_rate = 0, .target_pwm = 0,  .current_pwm = 0.0f  },
    { .active = false, .current_gpio = GPIO_NUM_5, .current_freq = 50, .pwm_slew_rate = 0, .target_pwm = 0,  .current_pwm = 0.0f  }
};

static bool slew_task_created = false;

static void pwm_slew_task(void *arg)
{
    const int update_rate_hz = 50; 
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / update_rate_hz);

    while (1)
    {
        for (int i = 0; i < NUM_PWM_CHANNELS; i++)
        {
            pwm_channel_state_t *st = &pwm_states[i];

            // Only process channels that have been explicitly initialized
            if (!st->active) continue;

            if (st->pwm_slew_rate > 0 && st->current_pwm != (float)st->target_pwm)
            {
                float delta = (float)st->pwm_slew_rate / update_rate_hz;
                
                if (st->current_pwm < st->target_pwm)
                {
                    st->current_pwm += delta;
                    if (st->current_pwm > st->target_pwm) st->current_pwm = st->target_pwm;
                }
                else if (st->current_pwm > st->target_pwm)
                {
                    st->current_pwm -= delta;
                    if (st->current_pwm < st->target_pwm) st->current_pwm = st->target_pwm;
                }
                
                ledc_set_duty(LEDC_LOW_SPEED_MODE, st->channel_num, (int)st->current_pwm);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, st->channel_num);
            }
        }
        vTaskDelay(delay_ticks);
    }
}

void pwm_driver_init(int channel)
{
    if (channel < 1 || channel > NUM_PWM_CHANNELS) return;
    int idx = channel - 1;

    // Do not re-allocate if already active
    if (pwm_states[idx].active) return;

    // 1. Request Hardware Resources from the Central Manager
    if (!ledc_manager_alloc_timer(pwm_states[idx].current_freq, &pwm_states[idx].timer_num))
    {
        ESP_LOGE(TAG, "PWM%d Init Failed: No free timers", channel);
        mesh_zigbee_send_text("ERR: NO FREE TIMERS");
        return; 
    }

    if (!ledc_manager_alloc_channel(&pwm_states[idx].channel_num))
    {
        ESP_LOGE(TAG, "PWM%d Init Failed: No free channels", channel);
        mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
        return; 
    }

    // 2. Mark as active
    pwm_states[idx].active = true;

    // 3. Apply the hardware configuration using the newly allocated resources
    pwm_driver_set_freq(channel, pwm_states[idx].current_freq);
    pwm_driver_set_pin(channel, pwm_states[idx].current_gpio);
    
    // Ensure the background task is running (only create it once globally)
    if (!slew_task_created)
    {
        xTaskCreate(pwm_slew_task, "pwm_slew_task", 2048, NULL, 5, NULL);
        slew_task_created = true;
    }

    ESP_LOGI(TAG, "PWM%d Driver initialized on GPIO %d (Timer %d, Channel %d)", 
             channel, pwm_states[idx].current_gpio, pwm_states[idx].timer_num, pwm_states[idx].channel_num);
}

void pwm_driver_set_pin(int channel, int pin)
{
    if (channel < 1 || channel > NUM_PWM_CHANNELS || pin < 0) return;
    int idx = channel - 1;

    if (pin != pwm_states[idx].current_gpio)
    {
        if (pwm_states[idx].active) {
            gpio_reset_pin(pwm_states[idx].current_gpio);
        }
        pwm_states[idx].current_gpio = pin;
    }

    // Only hit the hardware registers if the manager has given us a channel
    if (pwm_states[idx].active)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = pwm_states[idx].channel_num,
            .timer_sel      = pwm_states[idx].timer_num,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = pwm_states[idx].current_gpio,
            .duty           = pwm_states[idx].target_pwm, 
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        ESP_LOGI(TAG, "PWM%d pin configured to GPIO %d", channel, pwm_states[idx].current_gpio);
    }
}

void pwm_driver_set_freq(int channel, int freq)
{
    if (channel < 1 || channel > NUM_PWM_CHANNELS || freq <= 0) return;
    int idx = channel - 1;
    
    pwm_states[idx].current_freq = freq;

    // Only hit the hardware registers if the manager has given us a timer
    if (pwm_states[idx].active)
    {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = pwm_states[idx].timer_num,
            .duty_resolution  = LEDC_TIMER_10_BIT, 
            .freq_hz          = pwm_states[idx].current_freq,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
        ESP_LOGI(TAG, "PWM%d frequency configured to %d Hz", channel, pwm_states[idx].current_freq);
    }
}

void pwm_driver_set_slew(int channel, int rate)
{
    if (channel < 1 || channel > NUM_PWM_CHANNELS) return;
    int idx = channel - 1;

    if (rate < 0) rate = 0;
    pwm_states[idx].pwm_slew_rate = rate;
    ESP_LOGI(TAG, "PWM%d slew rate set to %d units/sec", channel, pwm_states[idx].pwm_slew_rate);
}

void pwm_driver_set_pwm(int channel, int raw_duty)
{
    if (channel < 1 || channel > NUM_PWM_CHANNELS) return;
    int idx = channel - 1;

    if (raw_duty < 0) raw_duty = 0;
    if (raw_duty > 1023) raw_duty = 1023;

    pwm_states[idx].target_pwm = raw_duty;

    if (pwm_states[idx].pwm_slew_rate <= 0)
    {
        pwm_states[idx].current_pwm = (float)pwm_states[idx].target_pwm;
        
        // Only push to hardware if the channel is actually active
        if (pwm_states[idx].active) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_states[idx].channel_num, pwm_states[idx].target_pwm);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_states[idx].channel_num);
            ESP_LOGI(TAG, "PWM%d snapped instantly to raw duty %d", channel, pwm_states[idx].target_pwm);
        }
    }
    else
    {
        ESP_LOGI(TAG, "PWM%d target set to %d (slewing at %d/sec)", channel, pwm_states[idx].target_pwm, pwm_states[idx].pwm_slew_rate);
    }
}

// --- Command Parsing and Execution ---

// Helper to extract channel number (1-4) from command string suffix
static int extract_channel(const char* cmd, const char* prefix)
{
    if (strncmp(cmd, prefix, strlen(prefix)) == 0)
    {
        int ch = cmd[strlen(prefix)] - '0';
        if (ch >= 1 && ch <= NUM_PWM_CHANNELS) {
            return ch;
        }
    }
    return -1;
}

bool pwm_driver_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    
    // Check if it matches any of the prefixes + a valid number (e.g. "pwm1", "SetupPWM4")
    if (extract_channel(cmd, "SetupPWM") != -1) return true;
    if (extract_channel(cmd, "pwmSlew") != -1) return true;
    if (extract_channel(cmd, "pwmPin") != -1) return true;
    if (extract_channel(cmd, "pwmFreq") != -1) return true;
    if (extract_channel(cmd, "pwm") != -1) return true;

    return false;
}

bool pwm_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    char reply[32];
    int ch;

    if ((ch = extract_channel(cmd, "SetupPWM")) != -1)
    {
        pwm_driver_init(ch);
        // Notice we don't automatically send OK here, because if it failed, 
        // pwm_driver_init already sent an ERR text to the coordinator.
        // If it succeeded, we can optionally send an OK. We'll check if active.
        if (pwm_states[ch-1].active) {
            snprintf(reply, sizeof(reply), "SETUP PWM%d OK", ch);
            if (!text_message) mesh_zigbee_send_text(reply);
        }
        return true;
    }
    else if ((ch = extract_channel(cmd, "pwmFreq")) != -1)
    {
        pwm_driver_set_freq(ch, value);
        snprintf(reply, sizeof(reply), "GOT PWMFREQ%d OK", ch);
        if (!text_message) mesh_zigbee_send_text(reply);
        return true;
    }
    else if ((ch = extract_channel(cmd, "pwmSlew")) != -1)
    {
        pwm_driver_set_slew(ch, value);
        snprintf(reply, sizeof(reply), "GOT PWMSLEW%d OK", ch);
        if (!text_message) mesh_zigbee_send_text(reply);
        return true;
    }
    else if ((ch = extract_channel(cmd, "pwmPin")) != -1)
    {
        pwm_driver_set_pin(ch, value);
        snprintf(reply, sizeof(reply), "GOT PWMPIN%d OK", ch);
        if (!text_message) mesh_zigbee_send_text(reply);
        return true;
    }
    else if ((ch = extract_channel(cmd, "pwm")) != -1) // Checked last to avoid false-matching longer prefixes
    {
        pwm_driver_set_pwm(ch, value);
        snprintf(reply, sizeof(reply), "GOT PWM%d OK", ch);
        if (!text_message) mesh_zigbee_send_text(reply);
        return true;
    }
    
    return false;
}