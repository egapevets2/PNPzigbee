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
 * @brief Checks if the shared I2C bus is currently physically latched (SDA or SCL stuck LOW).
 * @param[out] sda_level Pointer to store SDA line level (or NULL).
 * @param[out] scl_level Pointer to store SCL line level (or NULL).
 * @return true if SDA or SCL is stuck LOW (bus latched), false if both are HIGH (healthy/idle).
 */
bool i2c_manager_is_bus_latched(int *sda_level, int *scl_level);

/**
 * @brief Latched bus detector function for I2C peripherals.
 *        Evaluates an operation's return code and bus line states.
 *        If a bus failure or latched state is detected:
 *          1. Logs the failure with device context.
 *          2. Sends an upstream alert to the coordinator: "I2C BUS LATCHED: <device_name> (SDA=%d SCL=%d)"
 *          3. Attempts automatic bus recovery (clock pulsing + bus reset).
 *
 * @param device_name Name of the calling I2C peripheral (e.g. "SSD1306", "EXPAND8", etc.)
 * @param err Return code from the last I2C operation (e.g. i2c_master_transmit).
 * @return true if the bus was detected as latched/failed, false if healthy.
 */
bool i2c_manager_check_latched_bus(const char *device_name, esp_err_t err);

/**
 * @brief Attempts hardware bus recovery (generates SCL clock pulses to release stuck slaves,
 *        followed by STOP condition and master bus reset).
 * @return ESP_OK if recovered and lines are now HIGH, or ESP_FAIL if still latched.
 */
esp_err_t i2c_manager_recover_bus(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_MANAGER_H

