/*
 * Motor Driver Commands:
 * 
 * SetupMotorDriver mode freq
 *   - mode: 0 for DRV8833 (2-wire PWM), 1 for Cytron 10C (PWM/DIR)
 *   - freq: PWM frequency in Hz (e.g., 5000)
 *   Initializes the motor driver hardware with the specified mode and frequency.
 * 
 * MotorSpeed speed
 *   - speed: Target speed from -1000 (full reverse) to 1000 (full forward), 0 to stop.
 *   Sets the motor to the desired speed.
 * 
 * MotorSlew rate
 *   - rate: Slew rate in speed units per second. (0 = instant speed change).
 *   Configures how quickly the motor accelerates/decelerates to the target speed.
 */

#include "motor_driver.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include "mesh_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ledc_manager.h"

static const char *TAG = "MOTOR_DRIVER";

// --- MOTOR CONFIGURATION ---
#define MOTOR_PIN_1  GPIO_NUM_18 // DRV8833 IN1 or Cytron PWM
#define MOTOR_PIN_2  GPIO_NUM_20 // DRV8833 IN2 or Cytron DIR

#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // 0-1023 range

static ledc_timer_t motor_timer_num;
static ledc_channel_t motor_ch1_num;
static ledc_channel_t motor_ch2_num;
static bool motor_active = false;

static int motor_slew_rate = 0;
static int motor_target_speed = 0;
static float motor_current_speed = 0.0f;
static bool motor_slew_task_created = false;

static int motor_mode = 0; // 0: DRV8833, 1: Cytron 10C
static int motor_pwm_freq = 5000;

static void motor_hw_set_speed_internal(int16_t speed) 
{
    if (!motor_active) return;

    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (motor_mode == 0) // DRV8833
    {
        if (speed > 0) 
        {
            ledc_set_duty(LEDC_MODE, motor_ch2_num, 0);
            ledc_update_duty(LEDC_MODE, motor_ch2_num);
            
            ledc_set_duty(LEDC_MODE, motor_ch1_num, speed);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
        } 
        else if (speed < 0) 
        {
            ledc_set_duty(LEDC_MODE, motor_ch1_num, 0);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
            
            ledc_set_duty(LEDC_MODE, motor_ch2_num, -speed);
            ledc_update_duty(LEDC_MODE, motor_ch2_num);
        } 
        else 
        {
            ledc_set_duty(LEDC_MODE, motor_ch1_num, 0);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
            
            ledc_set_duty(LEDC_MODE, motor_ch2_num, 0);
            ledc_update_duty(LEDC_MODE, motor_ch2_num);
        }
    }
    else // Cytron 10C
    {
        if (speed > 0) 
        {
            gpio_set_level(MOTOR_PIN_2, 0);
            ledc_set_duty(LEDC_MODE, motor_ch1_num, speed);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
        } 
        else if (speed < 0) 
        {
            gpio_set_level(MOTOR_PIN_2, 1);
            ledc_set_duty(LEDC_MODE, motor_ch1_num, -speed);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
        } 
        else 
        {
            ledc_set_duty(LEDC_MODE, motor_ch1_num, 0);
            ledc_update_duty(LEDC_MODE, motor_ch1_num);
        }
    }
}

static void motor_slew_task(void *arg)
{
    const int update_rate_hz = 50; 
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000 / update_rate_hz);

    while (1)
    {
        if (motor_slew_rate > 0 && motor_current_speed != (float)motor_target_speed)
        {
            float delta = (float)motor_slew_rate / update_rate_hz;
            
            if (motor_current_speed < motor_target_speed)
            {
                motor_current_speed += delta;
                if (motor_current_speed > motor_target_speed) motor_current_speed = motor_target_speed;
            }
            else if (motor_current_speed > motor_target_speed)
            {
                motor_current_speed -= delta;
                if (motor_current_speed < motor_target_speed) motor_current_speed = motor_target_speed;
            }
            
            motor_hw_set_speed_internal((int16_t)motor_current_speed);

            // Notify coordinator when terminal speed is reached
            if (motor_current_speed == (float)motor_target_speed)
            {
                char msg[32];
                snprintf(msg, sizeof(msg), "Motor At %d", motor_target_speed);
                mesh_zigbee_send_text(msg);
            }
        }
        vTaskDelay(delay_ticks);
    }
}

void motor_driver_init(int mode, int freq) 
{
    if (motor_active) return;

    motor_mode = mode;
    motor_pwm_freq = freq > 0 ? freq : 5000;

    if (!ledc_manager_alloc_timer(motor_pwm_freq, &motor_timer_num)) {
        ESP_LOGE(TAG, "Motor Init Failed: No free timers");
        mesh_zigbee_send_text("ERR: NO FREE TIMERS");
        return;
    }

    if (motor_mode == 0) {
        if (!ledc_manager_alloc_channel(&motor_ch1_num) || !ledc_manager_alloc_channel(&motor_ch2_num)) {
            ESP_LOGE(TAG, "Motor Init Failed: No free channels");
            mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
            return;
        }
    } else {
        if (!ledc_manager_alloc_channel(&motor_ch1_num)) {
            ESP_LOGE(TAG, "Motor Init Failed: No free channels");
            mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
            return;
        }
    }

    motor_active = true;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = motor_timer_num,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = motor_pwm_freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_ch1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = motor_ch1_num,
        .timer_sel      = motor_timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PIN_1,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_ch1);

    if (motor_mode == 0) {
        ledc_channel_config_t ledc_ch2 = {
            .speed_mode     = LEDC_MODE,
            .channel        = motor_ch2_num,
            .timer_sel      = motor_timer_num,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MOTOR_PIN_2,
            .duty           = 0, 
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_ch2);
        ESP_LOGI(TAG, "DRV8833 initialized on PIN1:%d, PIN2:%d at %d Hz", MOTOR_PIN_1, MOTOR_PIN_2, motor_pwm_freq);
    } else {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << MOTOR_PIN_2),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(MOTOR_PIN_2, 0);
        ESP_LOGI(TAG, "Cytron 10C initialized on PWM:%d, DIR:%d at %d Hz", MOTOR_PIN_1, MOTOR_PIN_2, motor_pwm_freq);
    }

    if (!motor_slew_task_created) {
        xTaskCreate(motor_slew_task, "motor_slew_task", 2048, NULL, 5, NULL);
        motor_slew_task_created = true;
    }
}

void motor_set_slew(int rate)
{
    if (rate < 0) rate = 0;
    motor_slew_rate = rate;
    ESP_LOGI(TAG, "Motor slew rate set to %d units/sec", motor_slew_rate);
}

void motor_set_speed(int16_t speed) 
{
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    motor_target_speed = speed;

    if (motor_slew_rate <= 0)
    {
        motor_current_speed = (float)speed;
        motor_hw_set_speed_internal(speed);
    }
}

bool motor_driver_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "MotorSpeed") == 0 || strcmp(cmd, "MotorSlew") == 0 || strcmp(cmd, "SetupMotorDriver") == 0);
}

bool motor_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcmp(cmd, "MotorSpeed") == 0)
    {
        motor_set_speed(value);
        ESP_LOGI(TAG, "MotorSpeed set to %d", value);
        if (!text_message) mesh_zigbee_send_text("GOT MOTORSPEED OK");
        return true;
    }
    else if (strcmp(cmd, "MotorSlew") == 0)
    {
        motor_set_slew(value);
        if (!text_message) mesh_zigbee_send_text("GOT MOTORSLEW OK");
        return true;
    }
    else if (strcmp(cmd, "SetupMotorDriver") == 0)
    {
        motor_driver_init(value, value2);
        motor_set_speed(0);
        ESP_LOGI(TAG, "Motor Driver initialized");
        if (!text_message) mesh_zigbee_send_text("SETUP MOTOR OK");
        return true;
    }
    return false;
}