#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the LEDC peripheral for ESC (Electronic Speed Controller) based BLDC motor control.
 * @note This driver uses a single PWM signal, typically at 50Hz, to control the ESC.
 */
void BLDC_driver_init(void);

/**
 * @brief Sets the BLDC motor speed by controlling the ESC.
 * @param speed Speed value from -1000 (full reverse) to 1000 (full forward). 0 is stop.
 */
void BLDC_set_speed(int16_t speed);

/**
 * @brief Performs the arming sequence for the ESC.
 * Typically involves sending a neutral/stop signal for a few seconds.
 * This must be called after power-on before the motor will respond to speed commands.
 */
void BLDC_arm_esc(void);

/**
* @brief Checks if a given text command is handled by the BLDC driver.
*/
bool BLDC_driver_is_cmd(const char *cmd);

/**
* @brief Executes a BLDC-related command and sends the ACK to the coordinator.
*/
bool BLDC_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif