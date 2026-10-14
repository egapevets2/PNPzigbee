#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_DEFAULT_SDA_IO 22
#define I2C_DEFAULT_SCL_IO 23
#define I2C_DEFAULT_NUM    I2C_NUM_0
#define I2C_DEFAULT_FREQ   100000

/**
 * @brief Checks if the shared I2C master bus has been initialized.
 * @return true if active, false if pins 22/23 are unconfigured / general GPIO.
 */
bool i2c_manager_is_initialized(void);

/**
 * @brief Initializes the shared I2C bus on demand.
 * Configures SDA (GPIO 22) and SCL (GPIO 23) into I2C mode.
 * If already initialized, returns ESP_OK immediately.
 */
esp_err_t i2c_manager_init_bus(void);

/**
 * @brief Returns the active shared I2C master bus handle.
 * If the bus is not yet initialized, returns NULL.
 */
i2c_master_bus_handle_t i2c_manager_get_bus(void);

/**
 * @brief Backward-compatible helper: returns the shared bus handle,
 * initializing it on-demand if not already active.
 */
i2c_master_bus_handle_t get_shared_i2c_bus(void);

/**
 * @brief Releases the shared I2C bus and resets SCL/SDA pins back to standard GPIOs.
 */
esp_err_t i2c_manager_deinit_bus(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_MANAGER_H

