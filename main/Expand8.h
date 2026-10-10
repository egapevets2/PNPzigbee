#ifndef EXPAND8_H
#define EXPAND8_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCF8574_DEFAULT_ADDR   0x20
#define PCF8574A_DEFAULT_ADDR  0x38

/**
 * @brief Initialize the PCF8574 8-bit I/O expander on the shared I2C bus.
 *
 * @param i2c_addr 7-bit I2C slave address (e.g. 0x20-0x27 or 0x38-0x3F).
 *                 If 0 is passed, it automatically probes standard PCF8574/A addresses.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t expand8_init(uint8_t i2c_addr);

/**
 * @brief Write a single bit (0..7) on the PCF8574.
 *
 * @param bit Bit index [0..7]
 * @param val Logic level: 0 = low (strong pulldown), 1 = high (quasi-bidirectional pull-up / input)
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_write_bit(uint8_t bit, uint8_t val);

/**
 * @brief Read a single bit (0..7) from the PCF8574.
 *
 * @param bit Bit index [0..7]
 * @param[out] val Pointer to store the read logic level (0 or 1).
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_read_bit(uint8_t bit, uint8_t *val);

/**
 * @brief Write all 8 bits to the PCF8574 port.
 *
 * @param port_val 8-bit port value.
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_write_port(uint8_t port_val);

/**
 * @brief Read all 8 bits from the PCF8574 port.
 *
 * @param[out] port_val Pointer to store the 8-bit port value.
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_read_port(uint8_t *port_val);

/**
 * @brief Check if the command name belongs to Expand8.
 */
bool expand8_is_cmd(const char *cmd);

/**
 * @brief Execute an Expand8 command received over Zigbee / serial mesh.
 *
 * Supported commands:
 *   - SetupExpand8 [addr]
 *   - wrExpand8 K [val]  (where K in [0..7])
 *   - rdExpand8 K        (where K in [0..7])
 *
 * @param cmd Command string
 * @param value First numeric argument (bit index K or address)
 * @param value2 Second numeric argument (bit value 0 or 1, or -1 if omitted)
 * @param text_message True if dispatched from human-readable text packet
 * @return True if command was handled, false otherwise.
 */
bool expand8_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // EXPAND8_H

