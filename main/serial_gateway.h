#ifndef SERIAL_GATEWAY_H
#define SERIAL_GATEWAY_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Coordinator personality: USB-serial gateway, dynamic routing table,
 *        PuTTY bridge console, and network discovery sweep.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t serial_gateway_init(void);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_GATEWAY_H

