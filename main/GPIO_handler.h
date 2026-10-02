#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

void GPIO_Handler_Init(void);
void GPIO_ModeIn(int gpio_num);
void GPIO_ModeOut(int gpio_num);
void GPIO_SetVal(int gpio_num, int val);
void GPIO_Read(int gpio_num);

bool gpio_handler_is_cmd(const char *cmd);
bool gpio_handler_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // GPIO_HANDLER_H