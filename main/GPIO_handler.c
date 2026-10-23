/*
 * GPIO_handler.c - General Purpose Input/Output Pin Manager & Monitor
 *
 * Purpose:
 *   Manages dynamic GPIO pin mode configuration, real-time edge-triggered input change
 *   detection, and fail-safe RF drop disconnect indication.
 *   - Background polling task (gpio_poll_task) samples configured INPUT pins every 50ms
 *     and automatically pushes notifications upstream ("GPIO <num> CHANGED TO <state>")
 *     upon detecting logic transitions.
 *   - Disconnect monitor task (rfdrop_blink_task) flashes assigned RF drop blink pins at 4 Hz
 *     whenever communication with the coordinator is severed.
 *
 * Supported Commands:
 *   ModeGPIOin <gpio>
 *     - gpio: GPIO pin number to configure as input with internal pull-up.
 *     Starts monitoring the pin and reporting state changes.
 *
 *   ModeGPIOout <gpio>
 *     - gpio: GPIO pin number to configure as standard push-pull digital output (initial 0).
 *
 *   RFdropBlink <gpio> [0|1] / ModeRFdropBlink <gpio> [0|1]
 *     - gpio: GPIO pin number to assign as RF disconnect indicator (or negative to clear all).
 *     - 1 to enable, 0 to disable.
 *     When coordinator communication drops, flashes 4 times per second (4 Hz); stays low when healthy.
 *
 *   SetGPIOval <gpio> <val>
 *     - gpio: GPIO pin number.
 *     - val: Output level (0 or 1).
 *     Drives specified output pin (configures as output if not already set).
 *
 *   ReadGPIO <gpio>
 *     - gpio: GPIO pin number to sample.
 *     Reads instantaneous logic level.
 *     Replies "GPIO <gpio> IS <0|1>".
 *
 *   SetupGPIOhandler
 *     Initializes GPIO handler subsystem and launches background polling and blink worker tasks.
 *     Replies "SETUP GPIO OK".
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "UpstreamQ.h"
#include "GPIO_handler.h"
#include "mesh_zigbee.h"

#define N_GPIO 6

static const char *TAG = "GPIO_HANDLER";

typedef enum
{
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_RFDROP_BLINK
} GPIO_Mode_t;

typedef struct GPIO_handler
{
    int index; // Set to -1 if slot is unused
    GPIO_Mode_t mode;
    bool State;
    bool PastState;
    bool rf_drop_blink;
} GPIO_t;

GPIO_t GPIOlist[N_GPIO] = {
    {-1, GPIO_INPUT, false, false, false},
    {-1, GPIO_INPUT, false, false, false},
    {-1, GPIO_INPUT, false, false, false},
    {-1, GPIO_INPUT, false, false, false},
    {-1, GPIO_INPUT, false, false, false},
    {-1, GPIO_INPUT, false, false, false}
};

static int find_or_allocate_gpio(int gpio_num)
{
    for (int i = 0; i < N_GPIO; i++) {
        if (GPIOlist[i].index == gpio_num) return i;
    }
    for (int i = 0; i < N_GPIO; i++) {
        if (GPIOlist[i].index == -1) {
            GPIOlist[i].index = gpio_num;
            return i;
        }
    }
    ESP_LOGE(TAG, "No free slots in GPIOlist (Max %d)", N_GPIO);
    return -1;
}

void GPIO_ModeIn(int gpio_num)
{
    int idx = find_or_allocate_gpio(gpio_num);
    if (idx < 0) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    GPIOlist[idx].mode = GPIO_INPUT;
    GPIOlist[idx].rf_drop_blink = false;
    GPIOlist[idx].State = gpio_get_level(gpio_num);
    GPIOlist[idx].PastState = GPIOlist[idx].State;
    
    ESP_LOGI(TAG, "GPIO %d set to INPUT", gpio_num);
}

void GPIO_ModeOut(int gpio_num)
{
    int idx = find_or_allocate_gpio(gpio_num);
    if (idx < 0) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    GPIOlist[idx].mode = GPIO_OUTPUT;
    GPIOlist[idx].rf_drop_blink = false;
    GPIOlist[idx].State = 0;     
    GPIOlist[idx].PastState = 0;
    gpio_set_level(gpio_num, 0); 
    
    ESP_LOGI(TAG, "GPIO %d set to OUTPUT", gpio_num);
}

void GPIO_RFdropBlink(int gpio_num, int enable)
{
    GPIO_Handler_Init();

    if (gpio_num < 0) {
        for (int i = 0; i < N_GPIO; i++) {
            if (GPIOlist[i].index != -1 && (GPIOlist[i].mode == GPIO_RFDROP_BLINK || GPIOlist[i].rf_drop_blink)) {
                GPIOlist[i].mode = GPIO_OUTPUT;
                GPIOlist[i].rf_drop_blink = false;
                gpio_set_level(GPIOlist[i].index, 0);
            }
        }
        ESP_LOGI(TAG, "All RFDROP_BLINK pins disabled");
        return;
    }

    int idx = find_or_allocate_gpio(gpio_num);
    if (idx < 0) return;

    if (enable) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << gpio_num),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);

        GPIOlist[idx].mode = GPIO_RFDROP_BLINK;
        GPIOlist[idx].rf_drop_blink = true;
        GPIOlist[idx].State = 0;
        GPIOlist[idx].PastState = 0;
        gpio_set_level(gpio_num, 0);

        ESP_LOGI(TAG, "GPIO %d set to RFDROP_BLINK (flashes 4x/sec when disconnected)", gpio_num);
    } else {
        GPIOlist[idx].mode = GPIO_OUTPUT;
        GPIOlist[idx].rf_drop_blink = false;
        GPIOlist[idx].State = 0;
        GPIOlist[idx].PastState = 0;
        gpio_set_level(gpio_num, 0);

        ESP_LOGI(TAG, "GPIO %d RFDROP_BLINK disabled", gpio_num);
    }
}

void GPIO_SetVal(int gpio_num, int val)
{
    int idx = find_or_allocate_gpio(gpio_num);
    if (idx >= 0) {
        if (GPIOlist[idx].mode != GPIO_OUTPUT) {
            GPIO_ModeOut(gpio_num);
        }
        gpio_set_level(gpio_num, val);
        GPIOlist[idx].State = val;
        GPIOlist[idx].PastState = val;
        ESP_LOGI(TAG, "GPIO %d set to %d", gpio_num, val);
    }
}

void GPIO_Read(int gpio_num)
{
    int val = gpio_get_level(gpio_num);
    char msg[32];
    snprintf(msg, sizeof(msg), "GPIO %d IS %d", gpio_num, val);
    SendTheMessage(msg);
    ESP_LOGI(TAG, "Read request fulfilled: %s", msg);
}

static void gpio_poll_task(void *arg)
{
    char msg[32];
    while (1)
    {
        for (int i = 0; i < N_GPIO; i++)
        {
            if (GPIOlist[i].index != -1 && GPIOlist[i].mode == GPIO_INPUT)
            {
                bool current_state = gpio_get_level(GPIOlist[i].index);
                if (current_state != GPIOlist[i].PastState)
                {
                    GPIOlist[i].State = current_state;
                    GPIOlist[i].PastState = current_state;
                    
                    snprintf(msg, sizeof(msg), "GPIO %d CHANGED TO %d", GPIOlist[i].index, current_state);
                    SendTheMessage(msg);
                    ESP_LOGI(TAG, "State Change: %s", msg);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void rfdrop_blink_task(void *arg)
{
    bool blink_val = false;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(125));

        bool has_rfdrop_pin = false;
        for (int i = 0; i < N_GPIO; i++) {
            if (GPIOlist[i].index != -1 && (GPIOlist[i].mode == GPIO_RFDROP_BLINK || GPIOlist[i].rf_drop_blink)) {
                has_rfdrop_pin = true;
                break;
            }
        }

        if (!has_rfdrop_pin) {
            continue;
        }

        bool connected = mesh_zigbee_is_coordinator_connected();
        if (!connected) {
            // Connection is severed -> blink 4 times per second (125ms per half-cycle = 250ms period = 4 Hz)
            blink_val = !blink_val;
            for (int i = 0; i < N_GPIO; i++) {
                if (GPIOlist[i].index != -1 && (GPIOlist[i].mode == GPIO_RFDROP_BLINK || GPIOlist[i].rf_drop_blink)) {
                    gpio_set_level(GPIOlist[i].index, blink_val ? 1 : 0);
                }
            }
        } else {
            // Connection is healthy -> ensure pin is LOW (0)
            if (blink_val) {
                blink_val = false;
            }
            for (int i = 0; i < N_GPIO; i++) {
                if (GPIOlist[i].index != -1 && (GPIOlist[i].mode == GPIO_RFDROP_BLINK || GPIOlist[i].rf_drop_blink)) {
                    gpio_set_level(GPIOlist[i].index, 0);
                }
            }
        }
    }
}

void GPIO_Handler_Init(void)
{
    static bool task_created = false;
    if (!task_created) {
        for (int i = 0; i < N_GPIO; i++) {
            if (GPIOlist[i].index == -1) {
                // leave as unallocated
            }
        }
        xTaskCreate(gpio_poll_task, "gpio_poll_task", 2048, NULL, 5, NULL);
        xTaskCreate(rfdrop_blink_task, "rfdrop_blink_task", 2048, NULL, 4, NULL);
        task_created = true;
        ESP_LOGI(TAG, "GPIO Polling and RFdropBlink tasks initialized.");
    }
}

bool gpio_handler_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "ModeGPIOin") == 0 || strcmp(cmd, "ModeGPIOout") == 0 ||
            strcmp(cmd, "SetGPIOval") == 0 || strcmp(cmd, "ReadGPIO") == 0 ||
            strcmp(cmd, "SetupGPIOhandler") == 0 ||
            strcmp(cmd, "RFdropBlink") == 0 || strcmp(cmd, "ModeRFdropBlink") == 0);
}

bool gpio_handler_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcmp(cmd, "ModeGPIOin") == 0) {
        GPIO_ModeIn(value);
        if (!text_message) mesh_zigbee_send_text("GOT MODEIN OK");
        return true;
    } else if (strcmp(cmd, "ModeGPIOout") == 0) {
        GPIO_ModeOut(value);
        if (!text_message) mesh_zigbee_send_text("GOT MODEOUT OK");
        return true;
    } else if (strcmp(cmd, "RFdropBlink") == 0 || strcmp(cmd, "ModeRFdropBlink") == 0) {
        int enable = (value2 >= 0) ? value2 : 1;
        GPIO_RFdropBlink(value, enable);
        if (!text_message) mesh_zigbee_send_text("GOT RFDROPBLINK OK");
        return true;
    } else if (strcmp(cmd, "SetGPIOval") == 0) {
        GPIO_SetVal(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT SETGPIO OK");
        return true;
    } else if (strcmp(cmd, "ReadGPIO") == 0) {
        GPIO_Read(value);
        return true;
    } else if (strcmp(cmd, "SetupGPIOhandler") == 0) {
        GPIO_Handler_Init();
        ESP_LOGI(TAG, "GPIO Handler initialized");
        if (!text_message) mesh_zigbee_send_text("SETUP GPIO OK");
        return true;
    }
    return false;
}