/*
 * light_driver.c - Onboard Status LED Driver
 *
 * Purpose:
 *   Controls the Seeed Studio XIAO ESP32-C6 onboard yellow user LED on GPIO 15 (active-low).
 *   Provides initialization and low-level digital ON/OFF control functions invoked by
 *   the system blink commands and visual network indicators.
 *
 * Supported Commands:
 *   None (Internal Subsystem/Driver; user commands 'blink', 'on', 'off' are dispatched in esp_zb_light.c)
 */

#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_15 // onboard LED.

void Onboard_LED_control(bool power)
{
    // Seeed Studio XIAO ESP32-C6 onboard LED on GPIO 15 is Active LOW (0 = ON, 1 = OFF)
    gpio_set_level(LED_GPIO, power ? 0 : 1);
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


    gpio_set_level(LED_GPIO, initial_state ? 0 : 1);
}