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
    GPIO_OUTPUT
} GPIO_Mode_t;

typedef struct GPIO_handler
{
    int index; // Set to -1 if slot is unused
    GPIO_Mode_t mode;
    bool State;
    bool PastState;
} GPIO_t;

GPIO_t GPIOlist[N_GPIO] = {
    {-1, GPIO_INPUT, false, false},
    {-1, GPIO_INPUT, false, false},
    {-1, GPIO_INPUT, false, false},
    {-1, GPIO_INPUT, false, false},
    {-1, GPIO_INPUT, false, false},
    {-1, GPIO_INPUT, false, false}
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
    GPIOlist[idx].State = 0;     
    GPIOlist[idx].PastState = 0;
    gpio_set_level(gpio_num, 0); 
    
    ESP_LOGI(TAG, "GPIO %d set to OUTPUT", gpio_num);
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

void GPIO_Handler_Init(void)
{
    static bool task_created = false;
    for (int i = 0; i < N_GPIO; i++) {
        GPIOlist[i].index = -1;
    }
    if (!task_created) {
        xTaskCreate(gpio_poll_task, "gpio_poll_task", 2048, NULL, 5, NULL);
        task_created = true;
        ESP_LOGI(TAG, "GPIO Polling task initialized.");
    }
}

bool gpio_handler_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "ModeGPIOin") == 0 || strcmp(cmd, "ModeGPIOout") == 0 ||
            strcmp(cmd, "SetGPIOval") == 0 || strcmp(cmd, "ReadGPIO") == 0 ||
            strcmp(cmd, "SetupGPIOhandler") == 0);
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