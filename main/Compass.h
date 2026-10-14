#ifndef COMPASS_H
#define COMPASS_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_DEFAULT_I2C_ADDR 0x0D

/**
 * @brief Initializes the QMC5883L 3-axis digital compass on the shared I2C bus.
 * Configures the sensor in continuous measurement mode (200Hz ODR, 8G range, 512 OSR).
 * @param i2c_addr 7-bit I2C address (default: 0x0D / 13 decimal).
 */
esp_err_t compass_init(uint8_t i2c_addr);

/**
 * @brief Checks if QMC5883L is currently initialized.
 */
bool compass_is_initialized(void);

/**
 * @brief Reads 3-axis magnetic field vector (X, Y, Z) in raw LSBs.
 * @param x Pointer to store X-axis magnetic field.
 * @param y Pointer to store Y-axis magnetic field.
 * @param z Pointer to store Z-axis magnetic field.
 */
esp_err_t compass_read_mag(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Command router checks.
 */
bool compass_is_cmd(const char *cmd);

/**
 * @brief Executes compass commands (SetupCompass, GetCompass, ReadCompass).
 */
bool compass_execute_cmd(const char *cmd, int16_t value, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // COMPASS_H

