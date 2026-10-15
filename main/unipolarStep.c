#include "unipolarStep.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "mesh_zigbee.h"
#include "i2c_manager.h"

static const char *TAG = "UNIPOLAR_STEP";

// Pin assignments
static int s_pin_orange = UNIPOLAR_DEFAULT_PIN_ORANGE; // Coil A1
static int s_pin_pink   = UNIPOLAR_DEFAULT_PIN_PINK;   // Coil A2
static int s_pin_yellow = UNIPOLAR_DEFAULT_PIN_YELLOW; // Coil B1
static int s_pin_blue   = UNIPOLAR_DEFAULT_PIN_BLUE;   // Coil B2

// Step delay (ms)
static int s_step_delay_ms = UNIPOLAR_DEFAULT_DELAY_MS;

// Stepper state
static volatile int s_target_steps = 0;
static int s_current_step = 0;
static bool s_initialized = false;
static bool s_task_running = false;
static TaskHandle_t s_task_handle = NULL;

// 4-step full-step sequence (2 coils on for maximum torque)
// Order of phases: +A (Orange), +B (Yellow), -A (Pink), -B (Blue)
// Output mapping: [0]=Orange, [1]=Yellow, [2]=Pink, [3]=Blue
static const uint8_t s_full_step_seq[4][4] = {
    { 1, 1, 0, 0 },  // Step 0: Orange (+A) & Yellow (+B)
    { 0, 1, 1, 0 },  // Step 1: Yellow (+B) & Pink (-A)
    { 0, 0, 1, 1 },  // Step 2: Pink (-A) & Blue (-B)
    { 1, 0, 0, 1 }   // Step 3: Blue (-B) & Orange (+A)
};

static void delay_ms_precise(int ms)
{
    if (ms <= 0) return;
    if (ms >= 10) {
        int ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    } else {
        esp_rom_delay_us((uint32_t)ms * 1000);
    }
}

static void apply_current_step(void)
{
    gpio_set_level(s_pin_orange, s_full_step_seq[s_current_step][0]);
    gpio_set_level(s_pin_yellow, s_full_step_seq[s_current_step][1]);
    gpio_set_level(s_pin_pink,   s_full_step_seq[s_current_step][2]);
    gpio_set_level(s_pin_blue,   s_full_step_seq[s_current_step][3]);
}

void UnipolarStep_PowerDown(void)
{
    if (s_initialized) {
        gpio_set_level(s_pin_orange, 0);
        gpio_set_level(s_pin_pink, 0);
        gpio_set_level(s_pin_yellow, 0);
        gpio_set_level(s_pin_blue, 0);
    }
}

static void unipolar_worker_task(void *arg)
{
    ESP_LOGI(TAG, "Unipolar worker task started");
    while (s_task_running) {
        if (s_target_steps != 0) {
            int dir = (s_target_steps > 0) ? 1 : -1;
            
            // Advance phase index
            if (dir > 0) {
                s_current_step = (s_current_step + 1) % 4;
            } else {
                s_current_step = (s_current_step + 3) % 4;
            }
            
            apply_current_step();
            s_target_steps -= dir;
            
            delay_ms_precise(s_step_delay_ms);
            
            if (s_target_steps == 0) {
                // Done with step move: power down coils to avoid motor heating
                UnipolarStep_PowerDown();
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

void UnipolarStep_Init(void)
{
    // If I2C was active on pin 22, release it first so GPIO22 is a standard output
    if (i2c_manager_is_initialized()) {
        ESP_LOGI(TAG, "Releasing I2C bus to free GPIO22 for Unipolar Stepper");
        i2c_manager_deinit_bus();
    }

    gpio_reset_pin(s_pin_orange);
    gpio_reset_pin(s_pin_pink);
    gpio_reset_pin(s_pin_yellow);
    gpio_reset_pin(s_pin_blue);

    gpio_set_direction(s_pin_orange, GPIO_MODE_OUTPUT);
    gpio_set_direction(s_pin_pink, GPIO_MODE_OUTPUT);
    gpio_set_direction(s_pin_yellow, GPIO_MODE_OUTPUT);
    gpio_set_direction(s_pin_blue, GPIO_MODE_OUTPUT);

    UnipolarStep_PowerDown();
    s_initialized = true;

    if (!s_task_running) {
        s_task_running = true;
        xTaskCreate(unipolar_worker_task, "unipolar_task", 2048, NULL, 5, &s_task_handle);
    }

    ESP_LOGI(TAG, "Unipolar Stepper initialized: Coil A(Orange=Pin%d, Pink=Pin%d), Coil B(Yellow=Pin%d, Blue=Pin%d)",
             s_pin_orange, s_pin_pink, s_pin_yellow, s_pin_blue);
}

void UnipolarStep_SetPins(int orange, int pink, int yellow, int blue)
{
    s_pin_orange = orange;
    s_pin_pink = pink;
    s_pin_yellow = yellow;
    s_pin_blue = blue;
    if (s_initialized) {
        UnipolarStep_Init();
    }
}

void UnipolarStep_StepFwd(int steps)
{
    if (!s_initialized) {
        UnipolarStep_Init();
    }
    if (steps > 0) {
        s_target_steps += steps;
    }
}

void UnipolarStep_StepBkw(int steps)
{
    if (!s_initialized) {
        UnipolarStep_Init();
    }
    if (steps > 0) {
        s_target_steps -= steps;
    }
}

void UnipolarStep_Stop(void)
{
    s_target_steps = 0;
    UnipolarStep_PowerDown();
}

void UnipolarStep_SetDelayMs(int ms)
{
    s_step_delay_ms = (ms > 0) ? ms : UNIPOLAR_DEFAULT_DELAY_MS;
}

int UnipolarStep_GetDelayMs(void)
{
    return s_step_delay_ms;
}

bool UnipolarStep_IsBusy(void)
{
    return (s_target_steps != 0);
}

bool unipolarStep_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "SetupUnipolar") == 0 ||
            strcasecmp(cmd, "UniStepFwd") == 0 ||
            strcasecmp(cmd, "UniiStepBkw") == 0 ||
            strcasecmp(cmd, "UniStepBkw") == 0 ||
            strcasecmp(cmd, "UniStepStop") == 0 ||
            strcasecmp(cmd, "UniStepDelay") == 0);
}

bool unipolarStep_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcasecmp(cmd, "SetupUnipolar") == 0) {
        UnipolarStep_Init();
        if (!text_message) mesh_zigbee_send_text("SETUP UNIPOLAR OK");
        return true;
    } else if (strcasecmp(cmd, "UniStepFwd") == 0) {
        int steps = (value > 0) ? value : 1;
        UnipolarStep_StepFwd(steps);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPFWD %d OK", steps);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(cmd, "UniiStepBkw") == 0 || strcasecmp(cmd, "UniStepBkw") == 0) {
        int steps = (value > 0) ? value : 1;
        UnipolarStep_StepBkw(steps);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPBKW %d OK", steps);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(cmd, "UniStepStop") == 0) {
        UnipolarStep_Stop();
        if (!text_message) mesh_zigbee_send_text("GOT UNISTEPSTOP OK");
        return true;
    } else if (strcasecmp(cmd, "UniStepDelay") == 0) {
        UnipolarStep_SetDelayMs(value);
        if (!text_message) mesh_zigbee_send_text("GOT UNISTEPDELAY OK");
        return true;
    }
    return false;
}
