/*
 * I2Craw.h - Direct Low-Level I2C Register Access & Bus Scanner Interface
 *
 * Purpose:
 *   Declares command dispatchers, latched-bus detectors, and raw read/write/scan
 *   prototypes for direct low-level I2C communication on the shared I2C bus.
 */
#ifndef I2CRAW_H
#define I2CRAW_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Latched-bus detector for I2Craw raw register operations.
 *        Evaluates I2C transaction status and bus line states.
 *        If latched bus detected, sends upstream alert to coordinator and initiates recovery.
 */
bool I2Craw_check_latched_bus(esp_err_t err);

// Command routers
bool I2Craw_is_cmd(const char *cmd);
bool I2Craw_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // I2CRAW_H
