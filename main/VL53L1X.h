#ifndef VL53L1X_H
#define VL53L1X_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define VL53L1X_I2C_ADDR 0x29

esp_err_t vl53l1x_init(void);
esp_err_t vl53l1x_read_distance(uint16_t *distance);
void vl53l1x_task(void *arg);

// Command routers
bool vl53l1x_is_cmd(const char *cmd);
bool vl53l1x_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // VL53L1X_H