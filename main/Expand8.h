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
#define EXPAND8_MAX_INSTANCES  8
#define EXPAND8_POLL_MS        250

/**
 * @brief Initialize a PCF8574 8-bit I/O expander instance on the shared I2C bus.
 *        Supports up to EXPAND8_MAX_INSTANCES (2) concurrent chips at distinct I2C addresses.
 *
 * @param i2c_addr 7-bit I2C slave address (e.g. 0x20-0x27 or 0x38-0x3F).
 *                 If 0 is passed, defaults to PCF8574_DEFAULT_ADDR (0x20 / 32).
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t expand8_init(uint8_t i2c_addr);

/**
 * @brief Write a single bit (0..7) on a specific PCF8574 chip by I2C address.
 *
 * @param i2c_addr 7-bit I2C slave address (or 0 to use active instance)
 * @param bit Bit index [0..7]
 * @param val Logic level: 0 = low (strong pulldown), 1 = high (quasi-bidirectional pull-up / input)
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_write_bit_addr(uint8_t i2c_addr, uint8_t bit, uint8_t val);

/**
 * @brief Write a single bit (0..7) on the currently active PCF8574 instance.
 */
esp_err_t expand8_write_bit(uint8_t bit, uint8_t val);

/**
 * @brief Read a single bit (0..7) from a specific PCF8574 chip by I2C address.
 *
 * @param i2c_addr 7-bit I2C slave address (or 0 to use active instance)
 * @param bit Bit index [0..7]
 * @param[out] val Pointer to store the read logic level (0 or 1).
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_read_bit_addr(uint8_t i2c_addr, uint8_t bit, uint8_t *val);

/**
 * @brief Read a single bit (0..7) from the currently active PCF8574 instance.
 */
esp_err_t expand8_read_bit(uint8_t bit, uint8_t *val);

/**
 * @brief Write all 8 bits to a specific PCF8574 chip port by I2C address.
 *
 * @param i2c_addr 7-bit I2C slave address (or 0 to use active instance)
 * @param port_val 8-bit port value.
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_write_port_addr(uint8_t i2c_addr, uint8_t port_val);

/**
 * @brief Write all 8 bits to the currently active PCF8574 instance.
 */
esp_err_t expand8_write_port(uint8_t port_val);

/**
 * @brief Read all 8 bits from a specific PCF8574 chip port by I2C address.
 *
 * @param i2c_addr 7-bit I2C slave address (or 0 to use active instance)
 * @param[out] port_val Pointer to store the 8-bit port value.
 * @return ESP_OK on success, or error code.
 */
esp_err_t expand8_read_port_addr(uint8_t i2c_addr, uint8_t *port_val);

/**
 * @brief Read all 8 bits from the currently active PCF8574 instance.
 */
esp_err_t expand8_read_port(uint8_t *port_val);

/**
 * @brief Latched-bus detector for PCF8574/A Expand8.
 *        Evaluates I2C operation status and bus line states.
 *        If latched bus detected, sends upstream alert to coordinator and initiates recovery.
 */
bool expand8_check_latched_bus(esp_err_t err);

/**
 * @brief Check if the command name belongs to Expand8.
 */
bool expand8_is_cmd(const char *cmd);

/**
 * @brief Execute an Expand8 command received over ESP-NOW / serial mesh.
 *
 * Supported commands (optional [addr] selects between the 2 chip instances):
 *   - SetupExpand8 [addr]
 *   - wrExpand8 <bit> <val> [addr]   (or wrExpand8 <addr> <bit> <val>)
 *   - rdExpand8 <bit> [addr]         (or rdExpand8 <addr> <bit>)
 *   - setExpand8 <bit> [addr]        (or setExpand8 <addr> <bit>)
 *   - clrExpand8 <bit> [addr]        (or clrExpand8 <addr> <bit>)
 *   - wrExpand8byte <byte> [addr]
 *   - rdExpand8byte [addr]
 *
 * @param cmd Command string
 * @param value First numeric argument
 * @param value2 Second numeric argument (-1 if omitted)
 * @param value3 Third numeric argument (-1 if omitted)
 * @param text_message True if dispatched from human-readable text packet
 * @return True if command was handled, false otherwise.
 */
bool expand8_execute_cmd(const char *cmd, int16_t value, int16_t value2, int16_t value3, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // EXPAND8_H

