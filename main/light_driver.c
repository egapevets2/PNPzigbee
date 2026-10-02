// File: light_driver.c
// ... (keep includes except driver/ledc.h) ...

#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_15 // onboard LED.

void Onboard_LED_control(bool power)
{
    gpio_set_level(LED_GPIO, power ? 1 : 0);
}

void light_driver_init(bool initial_state)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    // TEMP TEST
    for (int i = 0; i < 5; i++)
    {
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    gpio_set_level(LED_GPIO, initial_state ? 0 : 1);
}