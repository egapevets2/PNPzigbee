/*
 * Rotary.c - Keyes KY-040 Rotary Encoder Driver Subsystem
 *
 * Purpose:
 *   Manages the Keyes KY-040 rotary encoder with integrated pushbutton (default pins: CLK=4, DT=5, SW=6).
 *   - Monitors CLK and DT lines using any-edge GPIO interrupts and a noise-immune quadrature
 *     finite state machine (Ben Buxton algorithm) that inherently cancels contact bounce and EMI spikes.
 *   - Monitors the SW momentary pushbutton with an any-edge GPIO interrupt and 30 ms settling
 *     debounce filter in rotary_task.
 *   - Dispatches upstream event messages to the Coordinator:
 *       "RotaryCW"          upon clockwise detent rotation
 *       "RotaryCCW"         upon counter-clockwise detent rotation
 *       "RotaryButtonDown"  upon pressing the encoder pushbutton (level == 0)
 *       "RotaryButtonUp"    upon releasing the encoder pushbutton (level == 1)
 *
 * Settable via Coordinator Commands:
 *   SetupRotary [clk] [dt] [sw]     - Configures pins (if given) and starts encoder ("SETUP ROTARY OK/FAIL")
 *   SetRotaryPins <clk> <dt> <sw>   - Sets all 3 pins and starts encoder ("SET ROTARY PINS OK/FAIL")
 *   SetRotaryCLK <pin>              - Configures CLK pin ("GOT ROTARYCLK OK")
 *   SetRotaryDT <pin>               - Configures DT pin ("GOT ROTARYDT OK")
 *   SetRotarySW <pin>               - Configures SW pushbutton pin ("GOT ROTARYSW OK")
 *   SetRotaryInvert <0|1>           - Inverts CW/CCW direction in software ("GOT ROTARYINVERT OK")
 *   SetRotaryHalfStep <0|1>         - 1 = half-step (default, 00 and 11 detents), 0 = full-step (11 detent)
 *   MonitorRotary <0|1>             - Starts/stops 10ms raw bit monitor ("RotaryBits ...", alias: StreamRotary)
 *   GetRotary / ReadRotary          - Replies "ROTARY CLK=<c> DT=<d> SW=<s> POS=<pos> SW_LVL=<0|1>"
 *   ResetRotaryPos [val]            - Resets cumulative position count (default 0, "GOT RESETROTARYPOS OK")
 *   StopRotary / DeinitRotary       - Stops encoder and releases GPIOs ("STOP ROTARY OK")
 */

#include "Rotary.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "mesh_zigbee.h"
#include "UpstreamQ.h"

static const char *TAG = "ROTARY";

// Configured GPIO pins
static int s_pin_clk = ROTARY_DEFAULT_PIN_CLK;
static int s_pin_dt  = ROTARY_DEFAULT_PIN_DT;
static int s_pin_sw  = ROTARY_DEFAULT_PIN_SW;

// Subsystem state
static volatile bool s_rotary_active = false;
static volatile int32_t s_rotary_position = 0;
static volatile bool s_invert_dir = false;
static volatile bool s_half_step = true;
static volatile uint8_t s_encoder_state = 0;
static volatile int64_t s_last_btn_press_time = 0;
static volatile bool s_btn_is_pressed = false;
static volatile bool s_rotary_monitor_raw = false;
static TaskHandle_t s_rotary_raw_task_handle = NULL;

// FreeRTOS queue and worker task
#define ROTARY_QUEUE_LEN 32
static QueueHandle_t s_rotary_queue = NULL;
static TaskHandle_t s_rotary_task_handle = NULL;

typedef enum {
    ROTARY_EVT_CW,
    ROTARY_EVT_CCW,
    ROTARY_EVT_BTN_EDGE
} rotary_internal_event_t;

/*
 * Ben Buxton Quadrature State Machine
 *
 * Full-step state table:
 * Emits CW or CCW step only when returning to the resting detent position (both pins 1).
 */
#define R_START      0x0
#define R_CW_FINAL   0x1
#define R_CW_BEGIN   0x2
#define R_CW_NEXT    0x3
#define R_CCW_BEGIN  0x4
#define R_CCW_FINAL  0x5
#define R_CCW_NEXT   0x6

#define DIR_NONE 0x00
#define DIR_CW   0x10
#define DIR_CCW  0x20

static const uint8_t s_ttable_full[7][4] = {
    // 00         01           10           11
    {R_START,     R_CW_BEGIN,  R_CCW_BEGIN, R_START},           // R_START
    {R_CW_NEXT,   R_START,     R_CW_FINAL,  R_START | DIR_CW},  // R_CW_FINAL
    {R_CW_NEXT,   R_CW_BEGIN,  R_START,     R_START},           // R_CW_BEGIN
    {R_CW_NEXT,   R_CW_BEGIN,  R_CW_FINAL,  R_START},           // R_CW_NEXT
    {R_CCW_NEXT,  R_START,     R_CCW_BEGIN, R_START},           // R_CCW_BEGIN
    {R_CCW_NEXT,  R_CCW_FINAL, R_START,     R_START | DIR_CCW}, // R_CCW_FINAL
    {R_CCW_NEXT,  R_CCW_FINAL, R_CCW_BEGIN, R_START}            // R_CCW_NEXT
};

/*
 * Half-step state table:
 * Emits events at both detent positions (00 and 11) for encoders with detents every half-cycle.
 */
#define HS_START        0x0
#define HS_CCW_BEGIN    0x1
#define HS_CW_BEGIN     0x2
#define HS_START_M      0x3
#define HS_CW_BEGIN_M   0x4
#define HS_CCW_BEGIN_M  0x5

static const uint8_t s_ttable_half[6][4] = {
    // 00                   01              10              11
    {HS_START_M,            HS_CW_BEGIN,    HS_CCW_BEGIN,   HS_START},
    {HS_START_M | DIR_CCW,  HS_START,       HS_CCW_BEGIN,   HS_START},
    {HS_START_M | DIR_CW,   HS_CW_BEGIN,    HS_START,       HS_START},
    {HS_START_M,            HS_CCW_BEGIN_M, HS_CW_BEGIN_M,  HS_START},
    {HS_START_M,            HS_START_M,     HS_CW_BEGIN_M,  HS_START | DIR_CW},
    {HS_START_M,            HS_CCW_BEGIN_M, HS_START_M,     HS_START | DIR_CCW}
};

/*
 * Rotary Encoder ISR (CLK and DT lines)
 */
static void IRAM_ATTR rotary_isr_handler(void *arg)
{
    if (!s_rotary_active || s_pin_clk < 0 || s_pin_dt < 0) return;

    // Read current pin levels: CLK is bit 1, DT is bit 0
    uint8_t pin_state = (gpio_get_level((gpio_num_t)s_pin_clk) << 1) | gpio_get_level((gpio_num_t)s_pin_dt);

    uint8_t table_val;
    if (s_half_step) {
        s_encoder_state = s_ttable_half[s_encoder_state & 0x0F][pin_state];
        table_val = s_encoder_state & 0x30;
    } else {
        s_encoder_state = s_ttable_full[s_encoder_state & 0x0F][pin_state];
        table_val = s_encoder_state & 0x30;
    }

    if (table_val != DIR_NONE && s_rotary_queue != NULL) {
        rotary_internal_event_t evt;
        if (table_val == DIR_CW) {
            evt = s_invert_dir ? ROTARY_EVT_CCW : ROTARY_EVT_CW;
        } else {
            evt = s_invert_dir ? ROTARY_EVT_CW : ROTARY_EVT_CCW;
        }

        BaseType_t higher_woken = pdFALSE;
        xQueueSendFromISR(s_rotary_queue, &evt, &higher_woken);
        if (higher_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/*
 * Pushbutton Switch ISR (SW line, active LOW, triggered on ANYEDGE)
 */
static void IRAM_ATTR rotary_btn_isr_handler(void *arg)
{
    if (!s_rotary_active || s_pin_sw < 0) return;

    if (s_rotary_queue != NULL) {
        rotary_internal_event_t evt = ROTARY_EVT_BTN_EDGE;
        BaseType_t higher_woken = pdFALSE;
        xQueueSendFromISR(s_rotary_queue, &evt, &higher_woken);
        if (higher_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/*
 * Background worker task: consumes events from queue and sends upstream notifications
 */
static void rotary_worker_task(void *arg)
{
    rotary_internal_event_t evt;
    ESP_LOGI(TAG, "Rotary worker task running.");

    while (s_rotary_active) {
        if (xQueueReceive(s_rotary_queue, &evt, pdMS_TO_TICKS(200)) == pdTRUE) {
            switch (evt) {
                case ROTARY_EVT_CW:
                    s_rotary_position++;
                    ESP_LOGI(TAG, "RotaryCW event (pos=%ld)", (long)s_rotary_position);
                    SendTheMessage("RotaryCW");
                    break;

                case ROTARY_EVT_CCW:
                    s_rotary_position--;
                    ESP_LOGI(TAG, "RotaryCCW event (pos=%ld)", (long)s_rotary_position);
                    SendTheMessage("RotaryCCW");
                    break;

                case ROTARY_EVT_BTN_EDGE:
                    // Wait 30ms for contact bounce to settle
                    vTaskDelay(pdMS_TO_TICKS(30));
                    if (s_pin_sw >= 0) {
                        int level = gpio_get_level((gpio_num_t)s_pin_sw);
                        bool pressed = (level == 0);
                        if (pressed != s_btn_is_pressed) {
                            s_btn_is_pressed = pressed;
                            if (pressed) {
                                ESP_LOGI(TAG, "RotaryButtonDown event (SW pressed / held)");
                                SendTheMessage("RotaryButtonDown");
                            } else {
                                ESP_LOGI(TAG, "RotaryButtonUp event (SW released)");
                                SendTheMessage("RotaryButtonUp");
                            }
                        }
                    }
                    break;
            }
        }
    }

    ESP_LOGI(TAG, "Rotary worker task terminating.");
    s_rotary_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t rotary_init(int pin_clk, int pin_dt, int pin_sw)
{
    // Validate pins
    if (!GPIO_IS_VALID_GPIO(pin_clk) || !GPIO_IS_VALID_GPIO(pin_dt) || !GPIO_IS_VALID_GPIO(pin_sw)) {
        ESP_LOGE(TAG, "Invalid GPIO pins specified: CLK=%d, DT=%d, SW=%d", pin_clk, pin_dt, pin_sw);
        return ESP_ERR_INVALID_ARG;
    }

    // If currently running, cleanly deinitialize previous pins/ISRs first
    if (s_rotary_active) {
        rotary_deinit();
    }

    s_pin_clk = pin_clk;
    s_pin_dt  = pin_dt;
    s_pin_sw  = pin_sw;

    ESP_LOGI(TAG, "Initializing Keyes KY-040 Rotary Encoder: CLK=GPIO%d, DT=GPIO%d, SW=GPIO%d",
             s_pin_clk, s_pin_dt, s_pin_sw);

    // Reset pins
    gpio_reset_pin((gpio_num_t)s_pin_clk);
    gpio_reset_pin((gpio_num_t)s_pin_dt);
    gpio_reset_pin((gpio_num_t)s_pin_sw);

    // Configure CLK and DT as inputs with internal pull-up and any-edge interrupt
    gpio_config_t clk_dt_conf = {
        .pin_bit_mask = (1ULL << s_pin_clk) | (1ULL << s_pin_dt),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&clk_dt_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CLK/DT GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    // Configure SW as input with internal pull-up and any-edge interrupt (for momentary press and release)
    gpio_config_t sw_conf = {
        .pin_bit_mask = (1ULL << s_pin_sw),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    err = gpio_config(&sw_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SW GPIO: %s", esp_err_to_name(err));
        return err;
    }

    // Install global GPIO ISR service if not already installed
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }

    // Attach ISR handlers
    err = gpio_isr_handler_add((gpio_num_t)s_pin_clk, rotary_isr_handler, (void *)(intptr_t)s_pin_clk);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add CLK ISR: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)s_pin_dt, rotary_isr_handler, (void *)(intptr_t)s_pin_dt);
    if (err != ESP_OK) {
        gpio_isr_handler_remove((gpio_num_t)s_pin_clk);
        ESP_LOGE(TAG, "Failed to add DT ISR: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)s_pin_sw, rotary_btn_isr_handler, (void *)(intptr_t)s_pin_sw);
    if (err != ESP_OK) {
        gpio_isr_handler_remove((gpio_num_t)s_pin_clk);
        gpio_isr_handler_remove((gpio_num_t)s_pin_dt);
        ESP_LOGE(TAG, "Failed to add SW ISR: %s", esp_err_to_name(err));
        return err;
    }

    // Create event queue if needed
    if (s_rotary_queue == NULL) {
        s_rotary_queue = xQueueCreate(ROTARY_QUEUE_LEN, sizeof(rotary_internal_event_t));
        if (s_rotary_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create rotary event queue");
            gpio_isr_handler_remove((gpio_num_t)s_pin_clk);
            gpio_isr_handler_remove((gpio_num_t)s_pin_dt);
            gpio_isr_handler_remove((gpio_num_t)s_pin_sw);
            return ESP_ERR_NO_MEM;
        }
    }

    // Reset initial state based on resting pin levels
    uint8_t init_pins = (gpio_get_level((gpio_num_t)s_pin_clk) << 1) | gpio_get_level((gpio_num_t)s_pin_dt);
    if (s_half_step) {
        s_encoder_state = (init_pins == 0x00) ? HS_START_M : HS_START;
    } else {
        s_encoder_state = R_START;
    }
    s_last_btn_press_time = 0;
    s_btn_is_pressed = (s_pin_sw >= 0 && gpio_get_level((gpio_num_t)s_pin_sw) == 0);
    s_rotary_active = true;

    // Launch background worker task
    if (s_rotary_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(rotary_worker_task, "rotary_task", 3072, NULL, 5, &s_rotary_task_handle);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to launch rotary worker task");
            s_rotary_active = false;
            gpio_isr_handler_remove((gpio_num_t)s_pin_clk);
            gpio_isr_handler_remove((gpio_num_t)s_pin_dt);
            gpio_isr_handler_remove((gpio_num_t)s_pin_sw);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Keyes KY-040 Rotary Encoder ready. CLK=%d DT=%d SW=%d", s_pin_clk, s_pin_dt, s_pin_sw);
    return ESP_OK;
}

esp_err_t rotary_deinit(void)
{
    if (!s_rotary_active) return ESP_OK;

    s_rotary_active = false;
    s_rotary_monitor_raw = false;

    if (s_rotary_raw_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (s_pin_clk >= 0) {
        gpio_isr_handler_remove((gpio_num_t)s_pin_clk);
        gpio_reset_pin((gpio_num_t)s_pin_clk);
    }
    if (s_pin_dt >= 0) {
        gpio_isr_handler_remove((gpio_num_t)s_pin_dt);
        gpio_reset_pin((gpio_num_t)s_pin_dt);
    }
    if (s_pin_sw >= 0) {
        gpio_isr_handler_remove((gpio_num_t)s_pin_sw);
        gpio_reset_pin((gpio_num_t)s_pin_sw);
    }

    // Allow worker task to terminate cleanly
    if (s_rotary_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_rotary_task_handle != NULL) {
            vTaskDelete(s_rotary_task_handle);
            s_rotary_task_handle = NULL;
        }
    }

    if (s_rotary_queue != NULL) {
        vQueueDelete(s_rotary_queue);
        s_rotary_queue = NULL;
    }

    ESP_LOGI(TAG, "Rotary subsystem deinitialized.");
    return ESP_OK;
}

void rotary_set_pin_clk(int pin)
{
    s_pin_clk = pin;
    if (s_rotary_active) {
        rotary_init(s_pin_clk, s_pin_dt, s_pin_sw);
    }
}

void rotary_set_pin_dt(int pin)
{
    s_pin_dt = pin;
    if (s_rotary_active) {
        rotary_init(s_pin_clk, s_pin_dt, s_pin_sw);
    }
}

void rotary_set_pin_sw(int pin)
{
    s_pin_sw = pin;
    if (s_rotary_active) {
        rotary_init(s_pin_clk, s_pin_dt, s_pin_sw);
    }
}

void rotary_get_pins(int *pin_clk, int *pin_dt, int *pin_sw)
{
    if (pin_clk) *pin_clk = s_pin_clk;
    if (pin_dt)  *pin_dt  = s_pin_dt;
    if (pin_sw)  *pin_sw  = s_pin_sw;
}

bool rotary_is_active(void)
{
    return s_rotary_active;
}

int32_t rotary_get_position(void)
{
    return s_rotary_position;
}

void rotary_set_position(int32_t pos)
{
    s_rotary_position = pos;
}

void rotary_set_invert(bool invert)
{
    s_invert_dir = invert;
}

void rotary_set_half_step(bool half_step)
{
    s_half_step = half_step;
    if (s_rotary_active && s_pin_clk >= 0 && s_pin_dt >= 0) {
        uint8_t pins = (gpio_get_level((gpio_num_t)s_pin_clk) << 1) | gpio_get_level((gpio_num_t)s_pin_dt);
        if (s_half_step) {
            s_encoder_state = (pins == 0x00) ? HS_START_M : HS_START;
        } else {
            s_encoder_state = R_START;
        }
    }
}

static void rotary_raw_task(void *arg)
{
    int last_bits = -1;
    int last_sw = -1;
    int64_t last_time = 0;

    ESP_LOGI(TAG, "Rotary raw bit monitor task started");

    while (s_rotary_monitor_raw && s_rotary_active) {
        if (s_pin_clk >= 0 && s_pin_dt >= 0) {
            int clk = gpio_get_level((gpio_num_t)s_pin_clk);
            int dt  = gpio_get_level((gpio_num_t)s_pin_dt);
            int sw  = (s_pin_sw >= 0) ? gpio_get_level((gpio_num_t)s_pin_sw) : -1;
            int bits = (clk << 1) | dt;

            if (bits != last_bits || sw != last_sw) {
                int64_t now = esp_timer_get_time();
                int64_t delta_ms = (last_time == 0) ? 0 : (now - last_time) / 1000;
                last_time = now;
                last_bits = bits;
                last_sw = sw;

                char msg[64];
                snprintf(msg, sizeof(msg), "RotaryBits %d%d clk=%d dt=%d sw=%d +%lldms",
                         clk, dt, clk, dt, sw, (long long)delta_ms);
                SendTheMessage(msg);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Rotary raw bit monitor task stopped");
    s_rotary_raw_task_handle = NULL;
    vTaskDelete(NULL);
}

void rotary_set_monitor_raw(bool enable)
{
    s_rotary_monitor_raw = enable;
    if (enable && s_rotary_active && s_rotary_raw_task_handle == NULL) {
        xTaskCreate(rotary_raw_task, "rot_raw_task", 3072, NULL, 5, &s_rotary_raw_task_handle);
    }
}

bool rotary_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "SetupRotary") == 0 ||
            strcasecmp(cmd, "SetRotaryPins") == 0 ||
            strcasecmp(cmd, "SetRotaryCLK") == 0 ||
            strcasecmp(cmd, "SetRotaryDT") == 0 ||
            strcasecmp(cmd, "SetRotarySW") == 0 ||
            strcasecmp(cmd, "SetRotaryInvert") == 0 ||
            strcasecmp(cmd, "SetRotaryHalfStep") == 0 ||
            strcasecmp(cmd, "MonitorRotary") == 0 ||
            strcasecmp(cmd, "StreamRotary") == 0 ||
            strcasecmp(cmd, "GetRotary") == 0 ||
            strcasecmp(cmd, "ReadRotary") == 0 ||
            strcasecmp(cmd, "ResetRotaryPos") == 0 ||
            strcasecmp(cmd, "StopRotary") == 0 ||
            strcasecmp(cmd, "DeinitRotary") == 0);
}

bool rotary_execute_cmd(const char *cmd, int16_t value, int16_t value2, int16_t value3, bool text_message)
{
    if (!cmd) return false;

    if (strcasecmp(cmd, "SetupRotary") == 0 || strcasecmp(cmd, "SetRotaryPins") == 0) {
        int clk = (value >= 0) ? value : s_pin_clk;
        int dt  = (value2 >= 0) ? value2 : s_pin_dt;
        int sw  = (value3 >= 0) ? value3 : s_pin_sw;

        esp_err_t err = rotary_init(clk, dt, sw);
        if (err == ESP_OK) {
            mesh_zigbee_send_text("SETUP ROTARY OK");
        } else {
            mesh_zigbee_send_text("SETUP ROTARY FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "SetRotaryCLK") == 0) {
        rotary_set_pin_clk(value);
        if (!text_message) mesh_zigbee_send_text("GOT ROTARYCLK OK");
        return true;
    }
    else if (strcasecmp(cmd, "SetRotaryDT") == 0) {
        rotary_set_pin_dt(value);
        if (!text_message) mesh_zigbee_send_text("GOT ROTARYDT OK");
        return true;
    }
    else if (strcasecmp(cmd, "SetRotarySW") == 0) {
        rotary_set_pin_sw(value);
        if (!text_message) mesh_zigbee_send_text("GOT ROTARYSW OK");
        return true;
    }
    else if (strcasecmp(cmd, "SetRotaryInvert") == 0) {
        rotary_set_invert(value != 0);
        if (!text_message) mesh_zigbee_send_text("GOT ROTARYINVERT OK");
        return true;
    }
    else if (strcasecmp(cmd, "SetRotaryHalfStep") == 0) {
        rotary_set_half_step(value != 0);
        if (!text_message) mesh_zigbee_send_text("GOT ROTARYHALFSTEP OK");
        return true;
    }
    else if (strcasecmp(cmd, "MonitorRotary") == 0 || strcasecmp(cmd, "StreamRotary") == 0) {
        bool enable = (value > 0);
        rotary_set_monitor_raw(enable);
        if (enable) {
            mesh_zigbee_send_text("MONITOR ROTARY ON");
        } else {
            mesh_zigbee_send_text("MONITOR ROTARY OFF");
        }
        return true;
    }
    else if (strcasecmp(cmd, "GetRotary") == 0 || strcasecmp(cmd, "ReadRotary") == 0) {
        char resp[64];
        int sw_lvl = (s_pin_sw >= 0) ? gpio_get_level((gpio_num_t)s_pin_sw) : -1;
        snprintf(resp, sizeof(resp), "ROTARY CLK=%d DT=%d SW=%d POS=%ld SW_LVL=%d",
                 s_pin_clk, s_pin_dt, s_pin_sw, (long)s_rotary_position, sw_lvl);
        mesh_zigbee_send_text(resp);
        return true;
    }
    else if (strcasecmp(cmd, "ResetRotaryPos") == 0) {
        rotary_set_position(value >= 0 ? value : 0);
        if (!text_message) mesh_zigbee_send_text("GOT RESETROTARYPOS OK");
        return true;
    }
    else if (strcasecmp(cmd, "StopRotary") == 0 || strcasecmp(cmd, "DeinitRotary") == 0) {
        rotary_deinit();
        mesh_zigbee_send_text("STOP ROTARY OK");
        return true;
    }

    return false;
}

