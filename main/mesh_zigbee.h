#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_TEXT_LEN 18
#define MESH_LINE_LEN 80

typedef struct
{
    char source[MESH_TEXT_LEN];
    char target[MESH_TEXT_LEN];
    char cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2; // <-- Added to support two-value commands
    char text[MESH_LINE_LEN];
} mesh_msg_t;

esp_err_t mesh_zigbee_init(void);
bool mesh_zigbee_send(mesh_msg_t msg);
bool mesh_zigbee_send_text(const char *text);
bool mesh_zigbee_send_cmd(const char *cmd, int16_t value);
bool mesh_zigbee_receive(mesh_msg_t *msg, TickType_t timeout);
bool mesh_zigbee_is_joined(void);
int16_t mesh_zigbee_last_lqi(void);
void mesh_zigbee_set_rf_antenna(bool external);
void mesh_zigbee_set_rf_pins(int pwr, int sel);
void mesh_zigbee_set_sniffer(bool enable);
bool mesh_zigbee_send_broadcast(const char *text);

#ifdef __cplusplus
}
#endif