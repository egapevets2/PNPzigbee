#ifndef APDS9930_H
#define APDS9930_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#define APDS9930_I2C_ADDR 0x39
 
esp_err_t apds9930_init(void);
esp_err_t apds9930_config(void);
esp_err_t apds9930_read_proximity(uint16_t *proximity);
void proximity_task(void *arg);

/**
 * @brief Latched-bus detector for APDS-9930 proximity sensor.
 *        Evaluates I2C transaction status and bus line states.
 *        If latched bus detected, sends upstream alert to coordinator and initiates recovery.
 */
bool apds9930_check_latched_bus(esp_err_t err);

// Command routers
bool proximity_is_cmd(const char *cmd);
bool proximity_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // APDS9930_H
