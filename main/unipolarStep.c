#include "unipolarStep.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "mesh_zigbee.h"
#include "i2c_manager.h"

static const char *TAG = "UNIPOLAR_STEP";

// -----------------------------------------------------------------------------
// Direct Port of the Official Arduino Stepper.h / Stepper.cpp Library for ESP-IDF
// with Acceleration / Deceleration Profiling
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

// Step delay and acceleration profile:
// Delay0: Start and end step delay in ms (UnitStepDelay0 / UniStepDelay0)
// Delay1: Constant velocity step delay in ms (UnitStepDelay1 / UniStepDelay1)
// AccelSteps: Number of steps to ramp between Delay0 and Delay1 (0 = no ramp)
static int s_delay0_ms = UNIPOLAR_DEFAULT_DELAY_MS;
static int s_delay1_ms = UNIPOLAR_DEFAULT_DELAY_MS;
static int s_accel_steps = 0;

// Stepping mode: 0 = Arduino Stepper.cpp 4-wire (default), 1 = 8-step half-step
static int s_mode = 0;

// Movement command structure
typedef struct {
    int dir;           // +1 (Fwd) or -1 (Bkw)
    int cruise_steps;  // Constant velocity steps
    int accel_steps;   // Ramp steps
    float delay0;      // ms
    float delay1;      // ms
} unipolar_move_t;

static QueueHandle_t s_move_queue = NULL;
static volatile bool s_move_active = false;
static volatile bool s_stop_requested = false;
static int s_step_number = 0;
static bool s_initialized = false;
static bool s_task_running = false;
static TaskHandle_t s_task_handle = NULL;

static void delay_ms_float(float ms)
{
    if (ms <= 0.0f) return;
    int64_t target_us = (int64_t)(ms * 1000.0f);
    int64_t start = esp_timer_get_time();

    if (target_us >= 25000) {
        int ticks = (int)((target_us - 15000) / 10000);
        if (ticks > 0) {
            vTaskDelay(ticks);
        }
    }
    while ((esp_timer_get_time() - start) < target_us && !s_stop_requested) {
        int64_t remaining_us = target_us - (esp_timer_get_time() - start);
        if (remaining_us > 1000) {
            esp_rom_delay_us(500);
            taskYIELD();
        } else if (remaining_us > 0) {
            esp_rom_delay_us((uint32_t)remaining_us);
            break;
        }
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
    ESP_LOGI(TAG, "Unipolar worker task started (Arduino Stepper core with Accel/Decel)");
    unipolar_move_t move;

    while (s_task_running) {
        if (s_move_queue && xQueueReceive(s_move_queue, &move, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_move_active = true;
            s_stop_requested = false;

            int a = (move.accel_steps > 0) ? move.accel_steps : 0;
            int n = (move.cruise_steps > 0) ? move.cruise_steps : 0;
            int total_steps = (2 * a) + n;

            float d0 = (move.delay0 > 0.0f) ? move.delay0 : (float)UNIPOLAR_DEFAULT_DELAY_MS;
            float d1 = (move.delay1 > 0.0f) ? move.delay1 : d0;

            ESP_LOGI(TAG, "Starting move: dir=%d, total=%d (accel=%d, cruise=%d, decel=%d), delay0=%.1fms, delay1=%.1fms",
                     move.dir, total_steps, a, n, a, d0, d1);

            for (int k = 0; k < total_steps && !s_stop_requested; k++) {
                // Increment or decrement step number exactly like Arduino Stepper.cpp:
                if (move.dir == 1) {
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

                float step_delay = d0;
                if (a > 0) {
                    if (k < a) {
                        // Acceleration phase: ramp delay from d0 down to d1
                        float progress = (a > 1) ? ((float)k / (float)(a - 1)) : 1.0f;
                        step_delay = d0 + progress * (d1 - d0);
                    } else if (k >= a + n) {
                        // Deceleration phase: ramp delay from d1 up to d0
                        int j = k - (a + n);
                        float progress = (a > 1) ? ((float)j / (float)(a - 1)) : 1.0f;
                        step_delay = d1 + progress * (d0 - d1);
                    } else {
                        // Constant velocity cruise phase
                        step_delay = d1;
                    }
                } else {
                    // AccelSteps == 0: run entire movement at d0
                    step_delay = d0;
                }

                delay_ms_float(step_delay);
            }

            // Done stepping: power down coils to avoid heating and idle current
            UnipolarStep_PowerDown();
            s_move_active = false;
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

    if (!s_move_queue) {
        s_move_queue = xQueueCreate(4, sizeof(unipolar_move_t));
    }

    s_initialized = true;

    if (!s_task_running) {
        s_task_running = true;
        // Priority 4 allows app_msg_task (priority 5) to preempt when messages arrive
        xTaskCreate(unipolar_worker_task, "unipolar_task", 3072, NULL, 4, &s_task_handle);
    }

    ESP_LOGI(TAG, "Unipolar Stepper initialized (Arduino Stepper core + Accel/Decel): Pin1(Orange)=%d, Pin2(Pink)=%d, Pin21(Yellow)=%d, Pin22(Blue)=%d",
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
    if (steps > 0 && s_move_queue) {
        unipolar_move_t move = {
            .dir = 1,
            .cruise_steps = steps,
            .accel_steps = s_accel_steps,
            .delay0 = (float)s_delay0_ms,
            .delay1 = (float)s_delay1_ms
        };
        xQueueSend(s_move_queue, &move, 0);
    }
}

void UnipolarStep_StepBkw(int steps)
{
    if (!s_initialized) {
        UnipolarStep_Init();
    }
    if (steps > 0 && s_move_queue) {
        unipolar_move_t move = {
            .dir = -1,
            .cruise_steps = steps,
            .accel_steps = s_accel_steps,
            .delay0 = (float)s_delay0_ms,
            .delay1 = (float)s_delay1_ms
        };
        xQueueSend(s_move_queue, &move, 0);
    }
}

void UnipolarStep_Stop(void)
{
    s_stop_requested = true;
    if (s_move_queue) {
        xQueueReset(s_move_queue);
    }
    UnipolarStep_PowerDown();
}

void UnipolarStep_SetDelayMs(int ms)
{
    s_delay0_ms = (ms > 0) ? ms : UNIPOLAR_DEFAULT_DELAY_MS;
    s_delay1_ms = s_delay0_ms;
}

int UnipolarStep_GetDelayMs(void)
{
    return s_delay0_ms;
}

void UnipolarStep_SetDelay0Ms(int ms)
{
    s_delay0_ms = (ms > 0) ? ms : UNIPOLAR_DEFAULT_DELAY_MS;
}

int UnipolarStep_GetDelay0Ms(void)
{
    return s_delay0_ms;
}

void UnipolarStep_SetDelay1Ms(int ms)
{
    s_delay1_ms = (ms > 0) ? ms : UNIPOLAR_DEFAULT_DELAY_MS;
}

int UnipolarStep_GetDelay1Ms(void)
{
    return s_delay1_ms;
}

void UnipolarStep_SetAccelSteps(int steps)
{
    s_accel_steps = (steps >= 0) ? steps : 0;
}

int UnipolarStep_GetAccelSteps(void)
{
    return s_accel_steps;
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
    if (s_move_active) return true;
    if (s_move_queue && uxQueueMessagesWaiting(s_move_queue) > 0) return true;
    return false;
}

static void sanitize_cmd_name(const char *src, char *dst, size_t max_len, int *has_eq_val, int16_t *eq_val)
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

bool unipolarStep_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    char name[32];
    sanitize_cmd_name(cmd, name, sizeof(name), NULL, NULL);
    return (strcasecmp(name, "SetupUnipolar") == 0 ||
            strcasecmp(name, "UniStepFwd") == 0 ||
            strcasecmp(name, "UniiStepBkw") == 0 ||
            strcasecmp(name, "UniStepBkw") == 0 ||
            strcasecmp(name, "UniStepStop") == 0 ||
            strcasecmp(name, "UnitStepDelay0") == 0 ||
            strcasecmp(name, "UniStepDelay0") == 0 ||
            strcasecmp(name, "UniStepDelay") == 0 ||
            strcasecmp(name, "UnitStepDelay1") == 0 ||
            strcasecmp(name, "UniStepDelay1") == 0 ||
            strcasecmp(name, "AccelSteps") == 0 ||
            strcasecmp(name, "UniAccelSteps") == 0 ||
            strcasecmp(name, "UniStepMode") == 0);
}

bool unipolarStep_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (!cmd) return false;
    char name[32];
    int has_eq_val = 0;
    int16_t eq_val = 0;
    sanitize_cmd_name(cmd, name, sizeof(name), &has_eq_val, &eq_val);
    if (has_eq_val) {
        value = eq_val;
    }

    if (strcasecmp(name, "SetupUnipolar") == 0) {
        UnipolarStep_Init();
        if (!text_message) mesh_zigbee_send_text("SETUP UNIPOLAR OK");
        return true;
    } else if (strcasecmp(name, "UniStepFwd") == 0) {
        int steps = (value > 0) ? value : 1;
        UnipolarStep_StepFwd(steps);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPFWD %d OK", steps);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(name, "UniiStepBkw") == 0 || strcasecmp(name, "UniStepBkw") == 0) {
        int steps = (value > 0) ? value : 1;
        UnipolarStep_StepBkw(steps);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPBKW %d OK", steps);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(name, "UniStepStop") == 0) {
        UnipolarStep_Stop();
        if (!text_message) mesh_zigbee_send_text("GOT UNISTEPSTOP OK");
        return true;
    } else if (strcasecmp(name, "UnitStepDelay0") == 0 || strcasecmp(name, "UniStepDelay0") == 0 || strcasecmp(name, "UniStepDelay") == 0) {
        UnipolarStep_SetDelay0Ms(value);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPDELAY0 %d OK", value);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(name, "UnitStepDelay1") == 0 || strcasecmp(name, "UniStepDelay1") == 0) {
        UnipolarStep_SetDelay1Ms(value);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT UNISTEPDELAY1 %d OK", value);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(name, "AccelSteps") == 0 || strcasecmp(name, "UniAccelSteps") == 0) {
        UnipolarStep_SetAccelSteps(value);
        if (!text_message) {
            char resp[32];
            snprintf(resp, sizeof(resp), "GOT ACCELSTEPS %d OK", value);
            mesh_zigbee_send_text(resp);
        }
        return true;
    } else if (strcasecmp(name, "UniStepMode") == 0) {
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
