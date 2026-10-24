/*
 * Rotary.h - Keyes KY-040 Rotary Encoder Driver Subsystem
 *
 * Supported Hardware:
 *   Keyes KY-040 Rotary Encoder breakout module.
 *   - CLK: Output phase A (quadrature signal)
 *   - DT:  Output phase B (quadrature signal)
 *   - SW:  Pushbutton switch (active LOW, momentary contact)
 *   - +:   VCC (3.3V)
 *   - GND: Ground
 *
 * Upstream Messages Sent to Coordinator:
 *   - "RotaryCW"            : Clockwise rotation step detected
 *   - "RotaryCCW"           : Counter-clockwise rotation step detected
 *   - "RotaryButtonPushed"  : Pushbutton switch pressed (debounced)
 *
 * Supported Commands from Coordinator / Serial:
 *   - SetupRotary [clk] [dt] [sw]
 *   - SetRotaryPins <clk> <dt> <sw>
 *   - SetRotaryCLK <pin>
 *   - SetRotaryDT <pin>
 *   - SetRotarySW <pin>
 *   - SetRotaryInvert <0|1>
 *   - SetRotaryHalfStep <0|1>
 *   - GetRotary / ReadRotary
 *   - ResetRotaryPos [val]
 *   - StopRotary / DeinitRotary
 */

#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default GPIO pins if none specified via coordinator commands
#define ROTARY_DEFAULT_PIN_CLK  4
#define ROTARY_DEFAULT_PIN_DT   5
#define ROTARY_DEFAULT_PIN_SW   6

/**
 * @brief Initialize the Keyes KY-040 Rotary Encoder subsystem with specified GPIOs.
 *
 * @param pin_clk GPIO pin connected to encoder CLK line (Phase A)
 * @param pin_dt  GPIO pin connected to encoder DT line (Phase B)
 * @param pin_sw  GPIO pin connected to encoder SW line (Pushbutton, active LOW)
 * @return ESP_OK on success, or appropriate ESP-IDF error code.
 */
esp_err_t rotary_init(int pin_clk, int pin_dt, int pin_sw);

/**
 * @brief Deinitialize the rotary encoder subsystem, removing ISRs and freeing resources.
 *
 * @return ESP_OK on success.
 */
esp_err_t rotary_deinit(void);

/**
 * @brief Set individual GPIO pin for CLK (Phase A).
 */
void rotary_set_pin_clk(int pin);

/**
 * @brief Set individual GPIO pin for DT (Phase B).
 */
void rotary_set_pin_dt(int pin);

/**
 * @brief Set individual GPIO pin for SW (Pushbutton switch).
 */
void rotary_set_pin_sw(int pin);

/**
 * @brief Retrieve current configured GPIO pin assignments.
 */
void rotary_get_pins(int *pin_clk, int *pin_dt, int *pin_sw);

/**
 * @brief Check if the rotary encoder subsystem is actively monitoring pins.
 */
bool rotary_is_active(void);

/**
 * @brief Get the cumulative encoder position count.
 */
int32_t rotary_get_position(void);

/**
 * @brief Set the cumulative encoder position count.
 */
void rotary_set_position(int32_t pos);

/**
 * @brief Configure software direction inversion (swaps CW and CCW).
 */
void rotary_set_invert(bool invert);

/**
 * @brief Configure full-step vs half-step decoding mode.
 *        Full-step triggers once per detent (at rest state 11).
 *        Half-step triggers at both 00 and 11 states.
 */
void rotary_set_half_step(bool half_step);

/**
 * @brief Check if the command name belongs to the Rotary subsystem.
 */
bool rotary_is_cmd(const char *cmd);

/**
 * @brief Execute a rotary encoder command received from coordinator / mesh.
 *
 * @param cmd Command string
 * @param value First numeric argument
 * @param value2 Second numeric argument (-1 if omitted)
 * @param value3 Third numeric argument (-1 if omitted)
 * @param text_message True if dispatched from human-readable text packet
 * @return True if handled, false otherwise.
 */
bool rotary_execute_cmd(const char *cmd, int16_t value, int16_t value2, int16_t value3, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // ROTARY_H

