#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mesh_serial_init(void);
int mesh_serial_read_byte(void);
void mesh_serial_write(const char *s);
void mesh_serial_writef(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
