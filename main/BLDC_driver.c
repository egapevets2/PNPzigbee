#include "BLDC_driver.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include "mesh_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ledc_manager.h"

static const char *TAG = "BLDC_DRIVER";

/*
 * This driver is designed to control a standard 3-wire Brushless DC (BLDC) motor
 * via an Electronic Speed Controller (ESC). It generates a 50Hz PWM signal
 * with a pulse width between 1ms (0% throttle) and 2ms (100% throttle).
 *
 * NOTE: Only one GPIO pin is required to control the ESC. By default, this
 * driver uses the pin defined as BLDC_ESC_PIN.
 */

// --- MOTOR CONFIGURATION ---
#define BLDC_ESC_PIN  GPIO_NUM_18 // The GPIO pin connected to the ESC signal wire.

#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_RES      LEDC_TIMER_16_BIT // Use 16-bit resolution for high precision
#define LEDC_FREQUENCY      50                // 50 Hz, standard for ESCs

// --- Pulse Width Constants (in microseconds) ---
#define ESC_MIN_PULSE_US 1000 // 1ms pulse for 0% throttle
#define ESC_MAX_PULSE_US 2000 // 2ms pulse for 100% throttle
#define ESC_NEUTRAL_PULSE_US 1500 // 1.5ms pulse for neutral/stop (Bidirectional ESCs)
#define ESC_PERIOD_US    (1000000 / LEDC_FREQUENCY) // 20000 us for 50Hz

// --- State Variables ---
static ledc_timer_t bldc_timer_num;
static ledc_channel_t bldc_channel_num;
static bool bldc_active = false;

static uint32_t speed_to_duty(int16_t speed) {
    if (speed < -1000) speed = -1000;
    if (speed > 1000) speed = 1000;

    // Map speed [-1000, 1000] to pulse width [MIN, MAX] centered around NEUTRAL
    uint32_t pulse_us;
    if (speed >= 0) {
        pulse_us = ESC_NEUTRAL_PULSE_US + ((uint32_t)speed * (ESC_MAX_PULSE_US - ESC_NEUTRAL_PULSE_US)) / 1000;
    } else {
        pulse_us = ESC_NEUTRAL_PULSE_US - ((uint32_t)(-speed) * (ESC_NEUTRAL_PULSE_US - ESC_MIN_PULSE_US)) / 1000;
    }

    // Convert pulse width in microseconds to LEDC duty cycle
    // Duty = (pulse_us / Period_us) * 2^Resolution
    uint64_t duty = ((uint64_t)pulse_us * (1 << LEDC_TIMER_RES)) / ESC_PERIOD_US;
    return (uint32_t)duty;
}

void BLDC_driver_init(void) {
    if (bldc_active) return;

    ESP_LOGI(TAG, "Initializing BLDC ESC driver on GPIO %d", BLDC_ESC_PIN);

    if (!ledc_manager_alloc_timer(LEDC_FREQUENCY, &bldc_timer_num)) {
        ESP_LOGE(TAG, "BLDC Init Failed: No free timers for 50Hz");
        mesh_zigbee_send_text("ERR: NO FREE TIMERS");
        return;
    }

    if (!ledc_manager_alloc_channel(&bldc_channel_num)) {
        ESP_LOGE(TAG, "BLDC Init Failed: No free channels");
        mesh_zigbee_send_text("ERR: NO FREE CHANNELS");
        return;
    }

    bldc_active = true;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = bldc_timer_num,
        .duty_resolution  = LEDC_TIMER_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = bldc_channel_num,
        .timer_sel      = bldc_timer_num,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BLDC_ESC_PIN,
        .duty           = speed_to_duty(0), // Start at 0% throttle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "BLDC Driver Initialized. Arm ESC before setting speed.");
}

void BLDC_set_speed(int16_t speed) {
    if (!bldc_active) {
        ESP_LOGW(TAG, "BLDC driver not active. Call SetupBLDC first.");
        return;
    }
    uint32_t duty = speed_to_duty(speed);
    ledc_set_duty(LEDC_MODE, bldc_channel_num, duty);
    ledc_update_duty(LEDC_MODE, bldc_channel_num);
    ESP_LOGI(TAG, "BLDC speed set to %d (Duty: %u)", speed, duty);
}

void BLDC_arm_esc(void) {
    if (!bldc_active) {
        ESP_LOGW(TAG, "BLDC driver not active. Call SetupBLDC first.");
        return;
    }
    ESP_LOGI(TAG, "Arming ESC: Sending neutral throttle (1500us) for 2 seconds...");
    BLDC_set_speed(0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "ESC should be armed now.");
    mesh_zigbee_send_text("BLDC ESC Armed");
}

bool BLDC_driver_is_cmd(const char *cmd) {
    if (!cmd) return false;
    return (strcmp(cmd, "BLDCspeed") == 0 || strcmp(cmd, "BLDCarm") == 0 || strcmp(cmd, "SetupBLDC") == 0);
}

bool BLDC_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message) {
    if (strcmp(cmd, "SetupBLDC") == 0) {
        BLDC_driver_init();
        if (!text_message) mesh_zigbee_send_text("SETUP BLDC OK");
        return true;
    } else if (strcmp(cmd, "BLDCarm") == 0) {
        BLDC_arm_esc();
        // Arm function is blocking and sends its own message.
        return true;
    } else if (strcmp(cmd, "BLDCspeed") == 0) {
        BLDC_set_speed(value);
        if (!text_message) mesh_zigbee_send_text("GOT BLDCSPEED OK");
        return true;
    }
    return false;
}
