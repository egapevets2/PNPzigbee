#ifndef UNIPOLAR_STEP_H
#define UNIPOLAR_STEP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default XIAO ESP32-C6 pin assignments matching the Adafruit Stepper constructor:
// Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
// Pin 8  -> XIAO Pin 1  (Orange) - motor_pin_1 (Coil A1)
// Pin 9  -> XIAO Pin 2  (Pink)   - motor_pin_2 (Coil A2)
// Pin 10 -> XIAO Pin 21 (Yellow) - motor_pin_3 (Coil B1)
// Pin 11 -> XIAO Pin 0  (Blue)   - motor_pin_4 (Coil B2)
// Center taps connected to 5V; XIAO pins drive gate-level N-channel MOSFETs.
#define UNIPOLAR_DEFAULT_PIN_ORANGE   1   // motor_pin_1 (Orange)
#define UNIPOLAR_DEFAULT_PIN_PINK     2   // motor_pin_2 (Pink)
#define UNIPOLAR_DEFAULT_PIN_YELLOW  21   // motor_pin_3 (Yellow)
#define UNIPOLAR_DEFAULT_PIN_BLUE     0   // motor_pin_4 (Blue)

// Default step delay in ms (10ms = 100 steps/sec, safe & strong for 28BYJ-48)
#define UNIPOLAR_DEFAULT_DELAY_MS    10

/**
 * @brief Initialize GPIOs (pins 1, 2, 21, 0) and background stepping task.
 */
void UnipolarStep_Init(void);

/**
 * @brief Configure custom GPIO pins if needed.
 */
void UnipolarStep_SetPins(int orange, int pink, int yellow, int blue);

/**
 * @brief Step forward by N steps (exact Arduino Stepper.step(+N)).
 * @param steps Number of steps to advance.
 */
void UnipolarStep_StepFwd(int steps);

/**
 * @brief Step backward by N steps (exact Arduino Stepper.step(-N)).
 * @param steps Number of steps to reverse.
 */
void UnipolarStep_StepBkw(int steps);

/**
 * @brief Stop motion immediately and de-energize coils.
 */
void UnipolarStep_Stop(void);

/**
 * @brief Set delay between steps in milliseconds (sets both Delay0 and Delay1).
 * @param ms Step delay (e.g. 5-40 ms).
 */
void UnipolarStep_SetDelayMs(int ms);

/**
 * @brief Get current delay between steps in milliseconds.
 */
int UnipolarStep_GetDelayMs(void);

/**
 * @brief Set start and end step delay in milliseconds (UnitStepDelay0 / UniStepDelay0).
 */
void UnipolarStep_SetDelay0Ms(int ms);

/**
 * @brief Get start/end step delay in milliseconds.
 */
int UnipolarStep_GetDelay0Ms(void);

/**
 * @brief Set constant velocity (cruise) step delay in milliseconds (UnitStepDelay1 / UniStepDelay1).
 */
void UnipolarStep_SetDelay1Ms(int ms);

/**
 * @brief Get constant velocity step delay in milliseconds.
 */
int UnipolarStep_GetDelay1Ms(void);

/**
 * @brief Set acceleration and deceleration ramp step count (AccelSteps).
 *        0 = disabled (immediate cruise speed).
 */
void UnipolarStep_SetAccelSteps(int steps);

/**
 * @brief Get acceleration ramp step count.
 */
int UnipolarStep_GetAccelSteps(void);

/**
 * @brief Set stepping mode: 0 = Arduino Stepper 4-wire (default), 1 = Half-step (8-step).
 */
void UnipolarStep_SetMode(int mode);

/**
 * @brief Get current stepping mode.
 */
int UnipolarStep_GetMode(void);

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
