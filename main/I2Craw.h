/*
I2Craw.h
*/
#ifndef I2CRAW_H
#define I2CRAW_H

#include <stdbool.h>
#include <stdint.h>

// Command routers
bool I2Craw_is_cmd(const char *cmd);
bool I2Craw_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // I2CRAW_H