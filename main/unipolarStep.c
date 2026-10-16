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

// -----------------------------------------------------------------------------
// Direct Port of the Official Arduino Stepper.h / Stepper.cpp Library for ESP-IDF
//
// Matches the Adafruit example constructor:
//   Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
//
// Pin mapping:
//   motor_pin_1 (Arduino Pin 8)  = XIAO Pin 1  (Orange) - Coil A1
//   motor_pin_2 (Arduino Pin 9)  = XIAO Pin 2  (Pink)   - Coil A2
//   motor_pin_3 (Arduino Pin 10) = XIAO Pin 21 (Yellow) - Coil B1
//   motor_pin_4 (Arduino Pin 11) = XIAO Pin 22 (Blue)   - Coil B2
// -----------------------------------------------------------------------------

static int s_pin_orange = UNIPOLAR_DEFAULT_PIN_ORANGE; // motor_pin_1
static int s_pin_pink   = UNIPOLAR_DEFAULT_PIN_PINK;   // motor_pin_2
static int s_pin_yellow = UNIPOLAR_DEFAULT_PIN_YELLOW; // motor_pin_3
static int s_pin_blue   = UNIPOLAR_DEFAULT_PIN_BLUE;   // motor_pin_4

// Step delay in milliseconds (default 10ms for reliable pull-in torque)
static int s_step_delay_ms = UNIPOLAR_DEFAULT_DELAY_MS;

// Stepping mode: 0 = Arduino Stepper.cpp 4-wire (default), 1 = 8-step half-step
static int s_mode = 0;

// Stepper state
static volatile int s_target_steps = 0;
static int s_step_number = 0;
static bool s_initialized = false;
static bool s_task_running = false;
static TaskHandle_t s_task_handle = NULL;

static void delay_ms_precise(int ms)
{
    if (ms <= 0) return;
    if (ms >= 10) {
        int ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    } else {
        esp_rom_delay_us((uint32_t)ms * 1000);
        taskYIELD();
    }
}

/**
 * @brief Exact stepMotor logic from Arduino Stepper.cpp for 4-wire stepper
 *
 * Case 0 (1010): motor_pin_1=1, motor_pin_2=0, motor_pin_3=1, motor_pin_4=0 (Orange & Yellow)
 * Case 1 (0110): motor_pin_1=0, motor_pin_2=1, motor_pin_3=1, motor_pin_4=0 (Pink & Yellow)
 * Case 2 (0101): motor_pin_1=0, motor_pin_2=1, motor_pin_3=0, motor_pin_4=1 (Pink & Blue)
 * Case 3 (1001): motor_pin_1=1, motor_pin_2=0, motor_pin_3=0, motor_pin_4=1 (Orange & Blue)
 */
static void stepMotor(int thisStep)
{
    switch (((thisStep % 4) + 4) % 4) {
      case 0:  // 1010
        gpio_set_level(s_pin_orange, 1);
        gpio_set_level(s_pin_pink,   0);
        gpio_set_level(s_pin_yellow, 1);
        gpio_set_level(s_pin_blue,   0);
        break;
      case 1:  // 0110
        gpio_set_level(s_pin_orange, 0);
        gpio_set_level(s_pin_pink,   1);
        gpio_set_level(s_pin_yellow, 1);
        gpio_set_level(s_pin_blue,   0);
        break;
      case 2:  // 0101
        gpio_set_level(s_pin_orange, 0);
        gpio_set_level(s_pin_pink,   1);
        gpio_set_level(s_pin_yellow, 0);
        gpio_set_level(s_pin_blue,   1);
        break;
      case 3:  // 1001
        gpio_set_level(s_pin_orange, 1);
        gpio_set_level(s_pin_pink,   0);
        gpio_set_level(s_pin_yellow, 0);
        gpio_set_level(s_pin_blue,   1);
        break;
    }
}

/**
 * @brief Optional 8-step half-step sequence for smooth motion
 */
static void stepMotorHalfStep(int thisStep)
{
    int s = ((thisStep % 8) + 8) % 8;
    static const uint8_t s_half_step_table[8][4] = {
        { 1, 0, 0, 0 },  // Step 0: Orange
        { 1, 0, 1, 0 },  // Step 1: Orange & Yellow
        { 0, 0, 1, 0 },  // Step 2: Yellow
        { 0, 1, 1, 0 },  // Step 3: Pink & Yellow
        { 0, 1, 0, 0 },  // Step 4: Pink
        { 0, 1, 0, 1 },  // Step 5: Pink & Blue
        { 0, 0, 0, 1 },  // Step 6: Blue
        { 1, 0, 0, 1 }   // Step 7: Orange & Blue
    };
    gpio_set_level(s_pin_orange, s_half_step_table[s][0]);
    gpio_set_level(s_pin_pink,   s_half_step_table[s][1]);
    gpio_set_level(s_pin_yellow, s_half_step_table[s][2]);
    gpio_set_level(s_pin_blue,   s_half_step_table[s][3]);
}

void UnipolarStep_PowerDown(void)
{
    if (s_initialized) {
        gpio_set_level(s_pin_orange, 0);
        gpio_set_level(s_pin_pink,   0);
        gpio_set_level(s_pin_yellow, 0);
        gpio_set_level(s_pin_blue,   0);
    }
}

static void unipolar_worker_task(void *arg)
{
    ESP_LOGI(TAG, "Unipolar worker task started (Arduino Stepper core)");
    while (s_task_running) {
        if (s_target_steps != 0) {
            int dir = (s_target_steps > 0) ? 1 : -1;

            // Increment or decrement step number exactly like Arduino Stepper.cpp:
            if (dir == 1) {
                s_step_number++;
                if (s_step_number >= 2000000000) {
                    s_step_number = 0;
                }
            } else {
                if (s_step_number <= 0) {
                    s_step_number = 2000000000;
                }
                s_step_number--;
            }

            if (s_mode == 1) {
                stepMotorHalfStep(s_step_number);
            } else {
                stepMotor(s_step_number);
            }

            s_target_steps -= dir;

            delay_ms_precise(s_step_delay_ms);

            if (s_target_steps == 0) {
                // Done stepping: power down coils to avoid heating and idle current
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
    gpio_set_direction(s_pin_pink,   GPIO_MODE_OUTPUT);
    gpio_set_direction(s_pin_yellow, GPIO_MODE_OUTPUT);
    gpio_set_direction(s_pin_blue,   GPIO_MODE_OUTPUT);

    UnipolarStep_PowerDown();
    s_initialized = true;

    if (!s_task_running) {
        s_task_running = true;
        // Priority 4 allows app_msg_task (priority 5) to preempt when messages arrive
        xTaskCreate(unipolar_worker_task, "unipolar_task", 2048, NULL, 4, &s_task_handle);
    }

    ESP_LOGI(TAG, "Unipolar Stepper initialized (Arduino Stepper core): Pin1(Orange)=%d, Pin2(Pink)=%d, Pin21(Yellow)=%d, Pin22(Blue)=%d",
             s_pin_orange, s_pin_pink, s_pin_yellow, s_pin_blue);
}

void UnipolarStep_SetPins(int orange, int pink, int yellow, int blue)
{
    s_pin_orange = orange;
    s_pin_pink   = pink;
    s_pin_yellow = yellow;
    s_pin_blue   = blue;
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

void UnipolarStep_SetMode(int mode)
{
    s_mode = (mode == 1) ? 1 : 0;
    s_step_number = 0;
}

int UnipolarStep_GetMode(void)
{
    return s_mode;
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
            strcasecmp(cmd, "UniStepDelay0") == 0 ||
            strcasecmp(cmd, "UniStepDelay") == 0 ||
            strcasecmp(cmd, "UniStepMode") == 0);
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
    } else if (strcasecmp(cmd, "UniStepDelay0") == 0 || strcasecmp(cmd, "UniStepDelay") == 0) {
        UnipolarStep_SetDelayMs(value);
        if (!text_message) mesh_zigbee_send_text("GOT UNISTEPDELAY0 OK");
        return true;
    } else if (strcasecmp(cmd, "UniStepMode") == 0) {
        UnipolarStep_SetMode(value);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPMODE %d OK", value);
            mesh_zigbee_send_text(resp);
        }
        return true;
    }
    return false;
}
