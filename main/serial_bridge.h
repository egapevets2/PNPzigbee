#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Hardware UART bridge.
 *
 * Phase 1:
 *   Zigbee coordinator text -> ESP32-C6 hardware TX pin
 *
 * Phase 2:
 *   ESP32-C6 hardware RX pin -> Zigbee coordinator text
 */
esp_err_t serial_bridge_init(void);
void serial_bridge_write(const char *s);
void serial_bridge_write_line(const char *s);

#ifdef __cplusplus
}
#endif
