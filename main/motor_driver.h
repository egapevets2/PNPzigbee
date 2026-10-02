#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the LEDC peripherals for the motor driver.
 * @param mode 0 for DRV8833 (2-wire PWM), 1 for Cytron 10C (PWM/DIR)
 * @param freq The PWM frequency in Hz
 */
void motor_driver_init(int mode, int freq);

/**
 * @brief Sets the slew rate for motor speed changes.
 * @param rate The slew rate in speed units per second. (0 = instant)
 */
void motor_set_slew(int rate);

/**
 * @brief Sets the slew rate for motor speed changes.
 * @param rate The slew rate in speed units per second. (0 = instant)
 */
void motor_set_slew(int rate);

/**
 * @brief Sets the motor speed and direction.
 * @param speed Range from -1000 (full reverse) to 1000 (full forward). 0 stops the motor.
 */
void motor_set_speed(int16_t speed);

/**
* @brief Checks if a given text command is handled by the Motor driver.
*/
bool motor_driver_is_cmd(const char *cmd);

/**
* @brief Executes a Motor-related command and sends the ACK to the coordinator.
*/
bool motor_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif