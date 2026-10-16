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
static int s_pin_orange = UNIPOLAR_DEFAULT_PIN_ORANGE; // Phase 1 (Coil A1)
static int s_pin_pink   = UNIPOLAR_DEFAULT_PIN_PINK;   // Phase 2 (Coil A2)
static int s_pin_yellow = UNIPOLAR_DEFAULT_PIN_YELLOW; // Phase 3 (Coil B1)
static int s_pin_blue   = UNIPOLAR_DEFAULT_PIN_BLUE;   // Phase 4 (Coil B2)

// Step delay (ms)
static int s_step_delay_ms = UNIPOLAR_DEFAULT_DELAY_MS;

// Stepping mode: 0 = Full-step (2-coil), 1 = Half-step (8-step), 2 = Wave drive (1-coil)
static int s_mode = 0;

// Stepper state
static volatile int s_target_steps = 0;
static int s_current_step = 0;
static bool s_initialized = false;
static bool s_task_running = false;
static TaskHandle_t s_task_handle = NULL;

// -----------------------------------------------------------------------------
// Adafruit Small Reduction Stepper (28BYJ-48 unipolar) Phase Sequences
// Stator pole circular sequence: Orange (1) -> Pink (2) -> Yellow (3) -> Blue (4)
// Output column mapping: [0]=Orange, [1]=Pink, [2]=Yellow, [3]=Blue
// -----------------------------------------------------------------------------

// Mode 0: 4-step full-step sequence (2 adjacent coils on for maximum torque)
static const uint8_t s_full_step_seq[4][4] = {
    { 1, 1, 0, 0 },  // Step 0: Orange (1) & Pink (2)
    { 0, 1, 1, 0 },  // Step 1: Pink (2) & Yellow (3)
    { 0, 0, 1, 1 },  // Step 2: Yellow (3) & Blue (4)
    { 1, 0, 0, 1 }   // Step 3: Blue (4) & Orange (1)
};

// Mode 1: 8-step half-step sequence (smoothest motion)
static const uint8_t s_half_step_seq[8][4] = {
    { 1, 0, 0, 0 },  // Step 0: Orange
    { 1, 1, 0, 0 },  // Step 1: Orange & Pink
    { 0, 1, 0, 0 },  // Step 2: Pink
    { 0, 1, 1, 0 },  // Step 3: Pink & Yellow
    { 0, 0, 1, 0 },  // Step 4: Yellow
    { 0, 0, 1, 1 },  // Step 5: Yellow & Blue
    { 0, 0, 0, 1 },  // Step 6: Blue
    { 1, 0, 0, 1 }   // Step 7: Blue & Orange
};

// Mode 2: 4-step wave drive (1 coil on for low power)
static const uint8_t s_wave_step_seq[4][4] = {
    { 1, 0, 0, 0 },  // Step 0: Orange
    { 0, 1, 0, 0 },  // Step 1: Pink
    { 0, 0, 1, 0 },  // Step 2: Yellow
    { 0, 0, 0, 1 }   // Step 3: Blue
};

static inline int get_seq_length(void)
{
    return (s_mode == 1) ? 8 : 4;
}

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

static void apply_current_step(void)
{
    const uint8_t *pat = NULL;
    if (s_mode == 1) {
        pat = s_half_step_seq[s_current_step % 8];
    } else if (s_mode == 2) {
        pat = s_wave_step_seq[s_current_step % 4];
    } else {
        pat = s_full_step_seq[s_current_step % 4];
    }
    gpio_set_level(s_pin_orange, pat[0]);
    gpio_set_level(s_pin_pink,   pat[1]);
    gpio_set_level(s_pin_yellow, pat[2]);
    gpio_set_level(s_pin_blue,   pat[3]);
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
    ESP_LOGI(TAG, "Unipolar worker task started");
    while (s_task_running) {
        if (s_target_steps != 0) {
            int dir = (s_target_steps > 0) ? 1 : -1;
            int total_steps = (s_target_steps > 0) ? s_target_steps : -s_target_steps;
            int n_steps = get_seq_length();

            // Pre-energize current step for 15ms to magnetically lock rotor before stepping
            apply_current_step();
            delay_ms_precise(15);

            int step_count = 0;
            int ramp_len = (total_steps > 32) ? 16 : (total_steps / 4);
            if (ramp_len < 2) ramp_len = 2;
            int start_delay = s_step_delay_ms + 10;
            if (start_delay < 15) start_delay = 15;

            while (s_target_steps != 0 && s_task_running) {
                // Advance phase index
                if (dir > 0) {
                    s_current_step = (s_current_step + 1) % n_steps;
                } else {
                    s_current_step = (s_current_step + (n_steps - 1)) % n_steps;
                }

                apply_current_step();
                s_target_steps -= dir;
                step_count++;

                // Calculate step delay with soft-start acceleration and deceleration ramps
                int current_delay = s_step_delay_ms;
                int rem_steps = total_steps - step_count;

                if (step_count < ramp_len) {
                    int diff = start_delay - s_step_delay_ms;
                    current_delay = s_step_delay_ms + (diff * (ramp_len - step_count)) / ramp_len;
                } else if (rem_steps < ramp_len && rem_steps >= 0) {
                    int diff = start_delay - s_step_delay_ms;
                    current_delay = s_step_delay_ms + (diff * (ramp_len - rem_steps)) / ramp_len;
                }

                delay_ms_precise(current_delay);
            }

            // Movement complete: power down coils to avoid heating and idle current
            UnipolarStep_PowerDown();
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

    ESP_LOGI(TAG, "Unipolar Stepper initialized: Coil 1(Orange=Pin%d), Coil 2(Pink=Pin%d), Coil 3(Yellow=Pin%d), Coil 4(Blue=Pin%d)",
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
    if (mode >= 0 && mode <= 2) {
        s_mode = mode;
        s_current_step = 0;
    }
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
    } else if (strcasecmp(cmd, "UniStepDelay") == 0) {
        UnipolarStep_SetDelayMs(value);
        if (!text_message) mesh_zigbee_send_text("GOT UNISTEPDELAY OK");
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
