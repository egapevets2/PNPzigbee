/*
 * mesh_zigbee.h - ESP-NOW Star Network Radio & Message Interface
 *
 * Purpose:
 *   Declares message structures (mesh_msg_t), RF parameters, radio initialization functions,
 *   non-blocking message transmit/receive queues, and Coordinator connection health metrics.
 */

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

/**
 * Silence timeout (ms) before declaring connection to coordinator lost.
 * Idle heartbeats occur every ~1.0–1.5s; 6000ms tolerates 3 missing heartbeats
 * (4 intervals * ~1.5s = 6.0s) before marking connection severed.
 */
#define MESH_COORDINATOR_SILENCE_TIMEOUT_MS     6000

/**
 * Maximum consecutive transmission failures to coordinator before declaring
 * connection severed. Tolerates 3 missing heartbeats (severs on 4th consecutive fail).
 */
#define MESH_COORDINATOR_MAX_CONSECUTIVE_FAILS  4

typedef struct
{
    char source[MESH_TEXT_LEN];
    char target[MESH_TEXT_LEN];
    char cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2; // <-- Added to support two-value commands
    int16_t lqi;    // <-- Link Quality Indicator (0..255) of received frame
    char text[MESH_LINE_LEN];
} mesh_msg_t;

esp_err_t mesh_radio_init(void);
esp_err_t mesh_zigbee_init(void);
bool mesh_zigbee_send(mesh_msg_t msg);
bool mesh_zigbee_send_text(const char *text);
bool mesh_zigbee_send_cmd(const char *cmd, int16_t value);
bool mesh_zigbee_receive(mesh_msg_t *msg, TickType_t timeout);
bool mesh_zigbee_is_joined(void);
bool mesh_zigbee_is_coordinator_connected(void);
void mesh_zigbee_record_coordinator_contact(void);
TickType_t mesh_zigbee_get_last_coord_contact_tick(void);
int16_t mesh_zigbee_last_lqi(void);
void mesh_zigbee_set_rf_antenna(bool external);
void mesh_zigbee_set_rf_pins(int pwr, int sel);
void mesh_zigbee_set_sniffer(bool enable);
bool mesh_zigbee_send_broadcast(const char *text);

#ifdef __cplusplus
}
#endif