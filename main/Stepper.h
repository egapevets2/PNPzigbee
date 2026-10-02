// File: Stepper.h

#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>

void Stepper_Init(void);

// Pin configurations
void Stepper_SetApGPIO(int pin);
void Stepper_SetAnGPIO(int pin);
void Stepper_SetBpGPIO(int pin);
void Stepper_SetBnGPIO(int pin);

// Movement and timing
void Stepper_SetMsPerStep(int ms);
void Stepper_Step(int steps);

// PWM Configurations
void Stepper_SetPWMFreq(int freq);
void Stepper_SetPWMDuty(int duty_percent);

// NEW: Holding Torque
void Stepper_SetHoldPWM(int duty_percent);

// NEW: Acceleration / Deceleration
void Stepper_SetAccel(int steps);
void Stepper_SetDecel(int steps);

// NEW: Stepping Modes
void Stepper_SetModeFullStep(void);
void Stepper_SetModeHalfStep(void);
void Stepper_SetModeMicroStep(void);

// Command routers
bool stepper_is_cmd(const char *cmd);
bool stepper_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // STEPPER_H