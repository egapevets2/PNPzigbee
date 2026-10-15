#ifndef UNIPOLAR_STEP_H
#define UNIPOLAR_STEP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default XIAO ESP32-C6 pin assignments for Adafruit Small Reduction Stepper:
// Coil A: Orange (Pin 1), Pink (Pin 2)
// Coil B: Yellow (Pin 21), Blue (Pin 22)
// Center taps connected to 5V; XIAO pins drive gate-level N-channel MOSFETs.
#define UNIPOLAR_DEFAULT_PIN_ORANGE   1   // Coil A1 (Orange)
#define UNIPOLAR_DEFAULT_PIN_PINK     2   // Coil A2 (Pink)
#define UNIPOLAR_DEFAULT_PIN_YELLOW  21   // Coil B1 (Yellow)
#define UNIPOLAR_DEFAULT_PIN_BLUE    22   // Coil B2 (Blue)

#define UNIPOLAR_DEFAULT_DELAY_MS     5   // Default delay per step (ms)

/**
 * @brief Initialize GPIOs (pins 1, 2, 21, 22) and background stepping task.
 */
void UnipolarStep_Init(void);

/**
 * @brief Configure custom GPIO pins if needed.
 */
void UnipolarStep_SetPins(int orange, int pink, int yellow, int blue);

/**
 * @brief Step forward by N steps.
 * @param steps Number of steps to advance.
 */
void UnipolarStep_StepFwd(int steps);

/**
 * @brief Step backward by N steps.
 * @param steps Number of steps to reverse.
 */
void UnipolarStep_StepBkw(int steps);

/**
 * @brief Stop motion immediately and de-energize coils.
 */
void UnipolarStep_Stop(void);

/**
 * @brief Set delay between steps in milliseconds.
 * @param ms Step delay (e.g. 3-10 ms).
 */
void UnipolarStep_SetDelayMs(int ms);

/**
 * @brief Get current delay between steps in milliseconds.
 */
int UnipolarStep_GetDelayMs(void);

/**
 * @brief Check if the stepper motor is currently moving.
 */
bool UnipolarStep_IsBusy(void);

/**
 * @brief Turn off all 4 MOSFET outputs to eliminate quiescent current.
 */
void UnipolarStep_PowerDown(void);

/**
 * @brief Check if a command string belongs to this unipolar stepper module.
 */
bool unipolarStep_is_cmd(const char *cmd);

/**
 * @brief Dispatch and execute a unipolar stepper command.
 */
bool unipolarStep_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // UNIPOLAR_STEP_H

