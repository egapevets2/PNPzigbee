#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void NeoPixel_SetGPIO(int gpio_num);
void NeoPixel_SetRed(int index, int val);
void NeoPixel_SetGreen(int index, int val);
void NeoPixel_SetBlue(int index, int val);
void NeoPixel_Update(void);

bool neopixel_is_cmd(const char *cmd);
bool neopixel_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // NEOPIXEL_H