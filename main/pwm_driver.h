// File: pwm_driver.h

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Supports 4 independent PWM channels (1-4)
#define NUM_PWM_CHANNELS 4

/**
* @brief Initialize a specific LEDC timer and channel for PWM control.
*
* @param channel The PWM channel (1-4)
*/
void pwm_driver_init(int channel);

/**
* @brief Set the GPIO pin for a specific PWM hardware output.
*
* @param channel The PWM channel (1-4)
* @param pin The GPIO number
*/
void pwm_driver_set_pin(int channel, int pin);

/**
* @brief Set the operating frequency of a specific PWM channel.
*
* @param channel The PWM channel (1-4)
* @param freq The frequency in Hz
*/
void pwm_driver_set_freq(int channel, int freq);

/**
* @brief Set the slew rate for PWM value changes on a specific channel.
*
* @param channel The PWM channel (1-4)
* @param rate The slew rate in PWM values per second. (0 = instant)
*/
void pwm_driver_set_slew(int channel, int rate);

/**
* @brief Set raw 10-bit PWM duty cycle for a specific channel.
*
* @param channel The PWM channel (1-4)
* @param raw_duty The raw duty cycle value (0-1023)
*/
void pwm_driver_set_pwm(int channel, int raw_duty);

/**
* @brief Checks if a given text command is handled by the PWM driver.
*
* @param cmd The command string
* @return true if it is a PWM command, false otherwise
*/
bool pwm_driver_is_cmd(const char *cmd);

/**
* @brief Executes a PWM-related command and sends the ACK to the coordinator.
*
* @param cmd The command string
* @param value The primary argument 
* @param value2 The secondary argument
* @param text_message Flag indicating if the message originated as text
* @return true if handled successfully
*/
bool pwm_driver_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
} // extern "C"
#endif