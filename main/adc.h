#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>

void ADC_Setup(int idx);
int ADC_GetCurrentValue(int idx);
void ADC_SetUpperThresh(int idx, int val);
void ADC_SetLowerThresh(int idx, int val);

// Command routers
bool adc_is_cmd(const char *cmd);
bool adc_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // ADC_H