#ifndef APDS9930_H
#define APDS9930_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#define APDS9930_I2C_ADDR 0x39

i2c_master_bus_handle_t get_shared_i2c_bus(void);
esp_err_t apds9930_init(void);
esp_err_t apds9930_config(void);
esp_err_t apds9930_read_proximity(uint16_t *proximity);
void proximity_task(void *arg);

// Command routers
bool proximity_is_cmd(const char *cmd);
bool proximity_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message);

#endif // APDS9930_H