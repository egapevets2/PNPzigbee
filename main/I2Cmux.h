/*
 * I2Cmux.h - TCA9548A 8-Channel I2C Multiplexer / Switch Driver
 *
 * Purpose:
 *   Drives Texas Instruments TCA9548A (0x70..0x77) 8-channel I2C switches
 *   over the shared I2C bus (i2c_manager). Allows dynamically choosing one
 *   of the 8 downstream I2C channels (0..7) to communicate with conflicting
 *   or multiplexed I2C target devices.
 *
 * Supported Commands:
 *   SetI2Cmux <X> [addr]  (or SetI2Cmux <addr> <X>)
 *     - X: Channel index [0..7] (or 1..8 for 1-based index).
 *          Pass -1 or 255 to disable all channels.
 *     - addr: (Optional) 7-bit I2C address (0x70..0x77, default: 0x70).
 *     Selects the specified I2C channel.
 *     Replies "SET I2CMUX OK" or "SET I2CMUX FAIL".
 *
 *   GetI2Cmux [addr] / ReadI2Cmux [addr]
 *     - addr: (Optional) 7-bit I2C address.
 *     Reads the current active channel from the TCA9548A control register.
 *     Replies "I2CMUX CH <X>", "I2CMUX NONE", or "I2CMUX MASK 0x<HEX>".
 *
 *   SetupI2Cmux [addr]
 *     - addr: (Optional) 7-bit I2C address (default: 0x70, or auto-probe).
 *     Initializes I2C bus and verifies TCA9548A presence.
 *     Replies "SETUP I2CMUX OK" or "SETUP I2CMUX FAIL".
 *
 *   SetI2CmuxMask <mask> [addr]
 *     - mask: Raw 8-bit channel mask (0x00..0xFF).
 *     Replies "SET I2CMUX OK" or "SET I2CMUX FAIL".
 */

#ifndef I2CMUX_H
#define I2CMUX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TCA9548A_DEFAULT_ADDR 0x70
#define TCA9548A_MIN_ADDR     0x70
#define TCA9548A_MAX_ADDR     0x77
#define TCA9548A_NUM_CHANNELS 8

/**
 * @brief Initialize the TCA9548A multiplexer on the shared I2C bus.
 *
 * @param i2c_addr 7-bit I2C slave address (0x70..0x77).
 *                 If 0, probes 0x70..0x77 or defaults to 0x70.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t i2cmux_init(uint8_t i2c_addr);

/**
 * @brief Select one of the 8 I2C channels (0..7) on the active multiplexer.
 *
 * @param channel Channel index [0..7]. Pass -1 or 255 to disable all channels.
 *                If 8 is passed, it is mapped to channel 7 (1-based index 8).
 * @return ESP_OK on success, or error code.
 */
esp_err_t i2cmux_select_channel(int8_t channel);

/**
 * @brief Select one of the 8 I2C channels (0..7) on a specific TCA9548A address.
 *
 * @param i2c_addr 7-bit I2C address (0 to use active/default).
 * @param channel Channel index [0..7], or -1 / 255 to disable all channels.
 * @return ESP_OK on success, or error code.
 */
esp_err_t i2cmux_select_channel_addr(uint8_t i2c_addr, int8_t channel);

/**
 * @brief Set raw 8-bit channel enable mask directly on the TCA9548A control register.
 *
 * @param i2c_addr 7-bit I2C address (0 to use active/default).
 * @param mask Bitmask where bit N enables channel N (0x00 = all disabled).
 * @return ESP_OK on success, or error code.
 */
esp_err_t i2cmux_set_mask_addr(uint8_t i2c_addr, uint8_t mask);

/**
 * @brief Read current 8-bit control register / channel mask from TCA9548A.
 *
 * @param i2c_addr 7-bit I2C address (0 to use active/default).
 * @param[out] mask Pointer to store the read 8-bit mask.
 * @return ESP_OK on success, or error code.
 */
esp_err_t i2cmux_get_mask_addr(uint8_t i2c_addr, uint8_t *mask);

/**
 * @brief Get currently selected channel on active mux (0..7, or -1 if none/multiple).
 *
 * @return Active channel index (0..7), or -1 if no channel or multiple channels are active.
 */
int8_t i2cmux_get_selected_channel(void);

/**
 * @brief Acquire / release the recursive I2Cmux lock to prevent background sensor tasks
 *        and foreground commands from changing the active mux channel mid-transaction.
 */
void i2cmux_lock(void);
void i2cmux_unlock(void);

/**
 * @brief Lock the I2Cmux, save the current channel mask, and switch to `channel` (0..7, or -1 for main bus).
 *        Must be paired with `i2cmux_restore_and_unlock()` if it returns ESP_OK.
 */
esp_err_t i2cmux_switch_and_lock(int8_t channel, uint8_t *saved_mask_out, bool *mux_switched_out);

/**
 * @brief Restore the saved channel mask (if modified) and release the I2Cmux lock.
 */
void i2cmux_restore_and_unlock(uint8_t saved_mask, bool mux_switched);

/**
 * @brief Latched-bus detector for TCA9548A multiplexer.
 *        Evaluates I2C transaction status and bus line states.
 *        If latched bus detected, sends upstream alert to coordinator and initiates recovery.
 */
bool i2cmux_check_latched_bus(esp_err_t err);

/**
 * @brief Check if the command name belongs to I2Cmux.
 */
bool i2cmux_is_cmd(const char *cmd);

/**
 * @brief Execute an I2Cmux command received over Zigbee / serial mesh.
 *
 * Supported commands:
 *   - SetI2Cmux <X> [addr]   (or SetI2Cmux <addr> <X>)
 *   - SetupI2Cmux [addr]
 *   - GetI2Cmux [addr] / ReadI2Cmux [addr]
 *   - SetI2CmuxMask <mask> [addr]
 *
 * @return true if command was handled, false otherwise.
 */
bool i2cmux_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // I2CMUX_H

