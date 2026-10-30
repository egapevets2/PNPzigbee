/*
 * discovery.h - Dynamic Node Discovery, Routing Table & Topology Management Interface
 *
 * Purpose:
 *   Defines frame formats (espnow_frame_t), routing table structures (routing_entry_t),
 *   discovery command constants, and API prototypes for node announcement, Coordinator
 *   routing table maintenance (copy-out query and locked update functions), hardware
 *   peer management, and endpoint coordinator loss watchdog timeouts.
 */

#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_TEXT_LEN           18
#define MESH_LINE_LEN           80
#define ROUTING_TABLE_MAX_NODES 25

#define DISCOVERY_CMD_HELLO        "HELLO"
#define DISCOVERY_CMD_HELLO_ACK    "HELLO_ACK"
#define DISCOVERY_CMD_SWEEP        "DISCOVER_ALL"

/** Watchdog: Maximum duration (ms) an endpoint will tolerate missing coordinator contact before rebooting */
#define DISCOVERY_COORDINATOR_TIMEOUT_MS  (180 * 1000) // 3 minutes

typedef struct __attribute__((packed)) {
    char    source[MESH_TEXT_LEN];
    char    target[MESH_TEXT_LEN];
    char    cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2;
    char    text[MESH_LINE_LEN];
} espnow_frame_t;

typedef struct {
    uint8_t     mac[ESP_NOW_ETH_ALEN];
    uint64_t    ieee;
    char        name[MESH_TEXT_LEN];
    uint16_t    short_addr;
    uint8_t     endpoint;
    bool        online;
    int16_t     last_lqi;
    int16_t     remote_lqi;
    TickType_t  last_seen_tick;
    bool        fresh_ping_expected;
    bool        fresh_ping_seen;
} routing_entry_t;

extern const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN];

/**
 * @brief Ensures an ESP-NOW peer is registered with proper channel and LR rate config.
 */
bool ensure_peer_exists(const uint8_t *mac);

/**
 * @brief Convert Wi-Fi RSSI in dBm to LQI (0..255).
 */
int16_t rssi_to_lqi(int8_t rssi);

/* =========================================================================
   Coordinator Dynamic Routing Table API (Copy-Out / Mutex-Protected)
   ========================================================================= */

/**
 * @brief Initialize the in-memory dynamic routing table and synchronization mutex.
 */
esp_err_t routing_table_init(void);

/**
 * @brief Find an entry by case-insensitive node name and copy it out while holding the mutex.
 * 
 * @param name Node name to search.
 * @param out_entry Buffer to receive the entry copy (optional, can be NULL if checking existence).
 * @return true if entry was found, false otherwise.
 */
bool routing_table_find_by_name(const char *name, routing_entry_t *out_entry);

/**
 * @brief Find an entry by 6-byte MAC address and copy it out while holding the mutex.
 * 
 * @param mac 6-byte MAC address.
 * @param out_entry Buffer to receive the entry copy (optional, can be NULL).
 * @return true if entry was found, false otherwise.
 */
bool routing_table_find_by_mac(const uint8_t *mac, routing_entry_t *out_entry);

/**
 * @brief Insert or update a node in the dynamic routing table, optionally copying it out.
 * 
 * @param name Node name.
 * @param mac 6-byte MAC address.
 * @param lqi Link Quality Indication.
 * @param out_entry Buffer to receive the entry copy (optional, can be NULL).
 * @return true on success, false on error.
 */
bool routing_table_add_or_update(const char *name, const uint8_t *mac, int16_t lqi, routing_entry_t *out_entry);

/**
 * @brief Get total number of registered nodes in the routing table.
 */
size_t routing_table_get_count(void);

/**
 * @brief Get node entry by table index (0 .. count - 1), copying it out while holding the mutex.
 * 
 * @param index 0-based index in the table.
 * @param out_entry Buffer to receive the entry copy.
 * @return true if index was valid and entry copied, false otherwise.
 */
bool routing_table_get_by_index(size_t index, routing_entry_t *out_entry);

/**
 * @brief Atomically update an entry's online status, last seen tick, and LQI upon RX.
 * 
 * @param mac 6-byte sender MAC address.
 * @param lqi Link Quality Indication.
 * @param out_entry Optional buffer to copy the updated entry into.
 * @return true if entry was found and updated, false otherwise.
 */
bool routing_table_update_rx(const uint8_t *mac, int16_t lqi, routing_entry_t *out_entry);

/**
 * @brief Atomically update an entry's remote LQI and mark fresh ping as seen upon PONG.
 * 
 * @param mac 6-byte sender MAC address.
 * @param remote_lqi Reported remote LQI from node.
 * @return true if entry was found and updated, false otherwise.
 */
bool routing_table_update_pong(const uint8_t *mac, int16_t remote_lqi);

/**
 * @brief Prepare routing table for a fresh network report by setting fresh_ping_expected = true
 *        and fresh_ping_seen = false across all current entries.
 * 
 * @return Number of expected nodes.
 */
int routing_table_prepare_fresh_report(void);

/**
 * @brief Finalize fresh network report: marks any entry that did not respond as offline,
 *        and clears fresh_ping flags.
 */
void routing_table_finalize_fresh_report(void);

/**
 * @brief Clear all entries in the routing table and delete ESP-NOW peers.
 */
void routing_table_clear(void);

/* =========================================================================
   Tri-Layer Discovery Protocol API
   ========================================================================= */

/**
 * @brief Coordinator: Initialize discovery and trigger Layer 2 startup sweep.
 */
esp_err_t discovery_coord_init(void);

/**
 * @brief Coordinator: Broadcast Layer 2 DISCOVER_ALL sweep across the RF medium.
 */
esp_err_t discovery_coord_sweep(void);

/**
 * @brief Coordinator: Process an incoming frame for discovery protocol interactions.
 * 
 * @param src_mac Sender MAC address.
 * @param frame Incoming ESP-NOW frame.
 * @param lqi Link Quality Indication.
 * @return true if frame was a discovery control frame and consumed, false otherwise.
 */
bool discovery_coord_handle_rx(const uint8_t *src_mac, const espnow_frame_t *frame, int16_t lqi);

/**
 * @brief End Device: Initialize discovery and launch Layer 1 announcement task.
 */
esp_err_t discovery_node_init(void);

/**
 * @brief End Device: Check if Layer 1 boot announcement has been acknowledged.
 */
bool discovery_node_is_announced(void);

/**
 * @brief End Device: Process an incoming frame for discovery protocol interactions.
 * 
 * @param src_mac Sender MAC address.
 * @param frame Incoming ESP-NOW frame.
 * @param lqi Link Quality Indication.
 * @return true if frame was a discovery control frame and consumed, false otherwise.
 */
bool discovery_node_handle_rx(const uint8_t *src_mac, const espnow_frame_t *frame, int16_t lqi);

#ifdef __cplusplus
}
#endif

#endif // DISCOVERY_H

