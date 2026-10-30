/*
 * esp_zb_light.h - Main Application Lifecycle & Core Dispatcher Header
 *
 * Purpose:
 *   Declares application lifecycle prototypes, device manufacturer identifiers,
 *   and entry points for Coordinator, End Device, and Unprovisioned node roles.
 */

#include <stdint.h>
#include <stdbool.h>
#include "light_driver.h"


/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET /* Customized model identifier */
