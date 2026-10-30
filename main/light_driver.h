/*
 * light_driver.h - Onboard Status LED Driver Header
 *
 * Purpose:
 *   Controls the Seeed Studio XIAO ESP32-C6 onboard yellow user LED on GPIO 15 (active-low).
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIGHT_DEFAULT_ON  1
#define LIGHT_DEFAULT_OFF 0

#define CONFIG_EXAMPLE_STRIP_LED_GPIO   8
#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 1

void Onboard_LED_control(bool power);
void light_driver_init(bool power);

#ifdef __cplusplus
} // extern "C"
#endif