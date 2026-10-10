#include "discovery.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "nvs_config.h"

static const char *TAG = "DISCOVERY";

const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// In-memory dynamic routing table
static routing_entry_t s_table[ROUTING_TABLE_MAX_NODES];
static size_t s_table_count = 0;
static SemaphoreHandle_t s_table_mutex = NULL;
static uint16_t s_next_short_addr = 0x0001;

// Node announcement state
static volatile bool s_node_announced = false;
static TaskHandle_t s_node_discovery_task_handle = NULL;

int16_t rssi_to_lqi(int8_t rssi)
{
    if (rssi <= -100) return 0;
    if (rssi >= -30)  return 255;
    return (int16_t)(((rssi + 100) * 255) / 70);
}

bool ensure_peer_exists(const uint8_t *mac)
{
    if (!mac) return false;
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = nvs_config_get_channel();
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add peer %02x:%02x:%02x:%02x:%02x:%02x: %s",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], esp_err_to_name(err));
        return false;
    }

    if (nvs_config_get_lr_mode()) {
        esp_now_rate_config_t rate_cfg = {
            .phymode = WIFI_PHY_MODE_LR,
            .rate = WIFI_PHY_RATE_LORA_250K,
            .ersu = false,
            .dcm = false,
        };
        esp_now_set_peer_rate_config(mac, &rate_cfg);
    }
    return true;
}

/* =========================================================================
   Coordinator Routing Table Implementation
   ========================================================================= */

esp_err_t routing_table_init(void)
{
    if (!s_table_mutex) {
        s_table_mutex = xSemaphoreCreateMutex();
    }
    if (!s_table_mutex) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_table_mutex, portMAX_DELAY);
    memset(s_table, 0, sizeof(s_table));
    s_table_count = 0;
    s_next_short_addr = 0x0001;
    xSemaphoreGive(s_table_mutex);

    ensure_peer_exists(s_broadcast_mac);
    return ESP_OK;
}

routing_entry_t *routing_table_find_by_name(const char *name)
{
    if (!name || !s_table_mutex) return NULL;

    xSemaphoreTake(s_table_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_table_count; i++) {
        if (strcasecmp(s_table[i].name, name) == 0) {
            xSemaphoreGive(s_table_mutex);
            return &s_table[i];
        }
    }
    xSemaphoreGive(s_table_mutex);
    return NULL;
}

routing_entry_t *routing_table_find_by_mac(const uint8_t *mac)
{
    if (!mac || !s_table_mutex) return NULL;

    xSemaphoreTake(s_table_mutex, portMAX_DELAY);
    for (size_t i = 0; i < s_table_count; i++) {
        if (memcmp(s_table[i].mac, mac, ESP_NOW_ETH_ALEN) == 0) {
            xSemaphoreGive(s_table_mutex);
            return &s_table[i];
        }
    }
    xSemaphoreGive(s_table_mutex);
    return NULL;
}

routing_entry_t *routing_table_add_or_update(const char *name, const uint8_t *mac, int16_t lqi)
{
    if (!name || !mac || !s_table_mutex) return NULL;

    xSemaphoreTake(s_table_mutex, portMAX_DELAY);

    routing_entry_t *entry = NULL;

    // Check if entry exists by name or by MAC
    for (size_t i = 0; i < s_table_count; i++) {
        if (strcasecmp(s_table[i].name, name) == 0 ||
            memcmp(s_table[i].mac, mac, ESP_NOW_ETH_ALEN) == 0) {
            entry = &s_table[i];
            break;
        }
    }

    if (entry) {
        // Update existing entry
        memcpy(entry->mac, mac, ESP_NOW_ETH_ALEN);
        strncpy(entry->name, name, sizeof(entry->name) - 1);
        entry->name[sizeof(entry->name) - 1] = '\0';
        entry->online = true;
        entry->last_lqi = lqi;
        entry->last_seen_tick = xTaskGetTickCount();
    } else {
        // Allocate slot
        if (s_table_count < ROUTING_TABLE_MAX_NODES) {
            entry = &s_table[s_table_count++];
        } else {
            // Table full: evict entry with oldest tick
            size_t oldest_idx = 0;
            TickType_t oldest_tick = s_table[0].last_seen_tick;
            for (size_t i = 1; i < ROUTING_TABLE_MAX_NODES; i++) {
                if (s_table[i].last_seen_tick < oldest_tick) {
                    oldest_tick = s_table[i].last_seen_tick;
                    oldest_idx = i;
                }
            }
            entry = &s_table[oldest_idx];
            ESP_LOGW(TAG, "Routing table full: evicting '%s'", entry->name);
        }

        memset(entry, 0, sizeof(*entry));
        memcpy(entry->mac, mac, ESP_NOW_ETH_ALEN);
        strncpy(entry->name, name, sizeof(entry->name) - 1);
        entry->name[sizeof(entry->name) - 1] = '\0';

        // EUI-64 IEEE address calculation
        entry->ieee = ((uint64_t)mac[0] << 56) |
                      ((uint64_t)mac[1] << 48) |
                      ((uint64_t)mac[2] << 40) |
                      (0xFFULL          << 32) |
                      (0xFEULL          << 24) |
                      ((uint64_t)mac[3] << 16) |
                      ((uint64_t)mac[4] << 8)  |
                      ((uint64_t)mac[5]);

        entry->short_addr = s_next_short_addr++;
        entry->endpoint = 10;
        entry->online = true;
        entry->last_lqi = lqi;
        entry->remote_lqi = -1;
        entry->last_seen_tick = xTaskGetTickCount();

        ESP_LOGI(TAG, "Dynamic Route Added: '%s' -> %02X:%02X:%02X:%02X:%02X:%02X (SHORT=0x%04X, LQI=%d)",
                 entry->name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], entry->short_addr, lqi);
    }

    xSemaphoreGive(s_table_mutex);

    ensure_peer_exists(mac);
    return entry;
}

size_t routing_table_get_count(void)
{
    size_t count = 0;
    if (s_table_mutex) {
        xSemaphoreTake(s_table_mutex, portMAX_DELAY);
        count = s_table_count;
        xSemaphoreGive(s_table_mutex);
    }
    return count;
}

routing_entry_t *routing_table_get_by_index(size_t index)
{
    routing_entry_t *entry = NULL;
    if (s_table_mutex) {
        xSemaphoreTake(s_table_mutex, portMAX_DELAY);
        if (index < s_table_count) {
            entry = &s_table[index];
        }
        xSemaphoreGive(s_table_mutex);
    }
    return entry;
}

void routing_table_clear(void)
{
    if (s_table_mutex) {
        xSemaphoreTake(s_table_mutex, portMAX_DELAY);
        memset(s_table, 0, sizeof(s_table));
        s_table_count = 0;
        xSemaphoreGive(s_table_mutex);
    }
}

/* =========================================================================
   Coordinator Tri-Layer Discovery Functions
   ========================================================================= */

esp_err_t discovery_coord_init(void)
{
    esp_err_t err = routing_table_init();
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Coordinator dynamic discovery engine initialized");
    return discovery_coord_sweep();
}

esp_err_t discovery_coord_sweep(void)
{
    espnow_frame_t frame = {0};
    strncpy(frame.source, "coordinator", sizeof(frame.source) - 1);
    strncpy(frame.target, "broadcast", sizeof(frame.target) - 1);
    strncpy(frame.cmd, DISCOVERY_CMD_SWEEP, sizeof(frame.cmd) - 1);
    strncpy(frame.text, DISCOVERY_CMD_SWEEP, sizeof(frame.text) - 1);

    ensure_peer_exists(s_broadcast_mac);
    esp_err_t err = esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Dispatched Layer 2 Discovery Sweep (DISCOVER_ALL)");
    } else {
        ESP_LOGW(TAG, "Failed to broadcast discovery sweep: %s", esp_err_to_name(err));
    }
    return err;
}

bool discovery_coord_handle_rx(const uint8_t *src_mac, const espnow_frame_t *frame, int16_t lqi)
{
    if (!src_mac || !frame) return false;

    // Check if frame is a HELLO announcement
    if (strcmp(frame->cmd, DISCOVERY_CMD_HELLO) == 0 ||
        strcmp(frame->text, DISCOVERY_CMD_HELLO) == 0 ||
        strncmp(frame->text, "HELLO", 5) == 0) {

        routing_table_add_or_update(frame->source, src_mac, lqi);

        // Send unicast HELLO_ACK back to sender
        espnow_frame_t ack = {0};
        strncpy(ack.source, "coordinator", sizeof(ack.source) - 1);
        strncpy(ack.target, frame->source, sizeof(ack.target) - 1);
        strncpy(ack.cmd, DISCOVERY_CMD_HELLO_ACK, sizeof(ack.cmd) - 1);
        strncpy(ack.text, DISCOVERY_CMD_HELLO_ACK, sizeof(ack.text) - 1);
        ack.value = lqi;

        ensure_peer_exists(src_mac);
        esp_now_send(src_mac, (const uint8_t *)&ack, sizeof(ack));
        ESP_LOGI(TAG, "Dispatched HELLO_ACK to '%s' (%02X:%02X:%02X:%02X:%02X:%02X, LQI=%d)",
                 frame->source, src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5], lqi);
        return true;
    }

    // For any other frame from a node, automatically learn / update its routing entry
    if (frame->source[0] != '\0' && strcasecmp(frame->source, "coordinator") != 0 &&
        strcasecmp(frame->source, "unknown") != 0) {
        routing_table_add_or_update(frame->source, src_mac, lqi);
    }

    return false;
}

/* =========================================================================
   End Device Tri-Layer Discovery Functions
   ========================================================================= */

static void discovery_node_task(void *arg)
{
    const uint32_t backoff_delays_ms[] = {500, 1000, 2000, 5000, 10000};
    const size_t num_backoffs = sizeof(backoff_delays_ms) / sizeof(backoff_delays_ms[0]);

    ESP_LOGI(TAG, "Layer 1 Announcement Engine started for node '%s'", nvs_config_get_name());

    // Layer 1: Exponential backoff announcement sequence
    for (size_t i = 0; i < num_backoffs && !s_node_announced; i++) {
        espnow_frame_t frame = {0};
        strncpy(frame.source, nvs_config_get_name(), sizeof(frame.source) - 1);
        strncpy(frame.target, "coordinator", sizeof(frame.target) - 1);
        strncpy(frame.cmd, DISCOVERY_CMD_HELLO, sizeof(frame.cmd) - 1);
        strncpy(frame.text, DISCOVERY_CMD_HELLO, sizeof(frame.text) - 1);

        ensure_peer_exists(s_broadcast_mac);
        esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
        ESP_LOGI(TAG, "Dispatched Layer 1 HELLO broadcast (attempt %d, backoff %lums)",
                 (int)(i + 1), (unsigned long)backoff_delays_ms[i]);

        vTaskDelay(pdMS_TO_TICKS(backoff_delays_ms[i]));
    }

    // Steady-state background heartbeat (every 30 seconds)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        espnow_frame_t frame = {0};
        strncpy(frame.source, nvs_config_get_name(), sizeof(frame.source) - 1);
        strncpy(frame.target, "coordinator", sizeof(frame.target) - 1);
        strncpy(frame.cmd, DISCOVERY_CMD_HELLO, sizeof(frame.cmd) - 1);
        strncpy(frame.text, DISCOVERY_CMD_HELLO, sizeof(frame.text) - 1);

        uint8_t coord_mac[6] = {0};
        if (nvs_config_get_coord_mac(coord_mac) == ESP_OK && coord_mac[0] != 0xFF) {
            ensure_peer_exists(coord_mac);
            esp_now_send(coord_mac, (const uint8_t *)&frame, sizeof(frame));
        } else {
            ensure_peer_exists(s_broadcast_mac);
            esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
        }
        ESP_LOGD(TAG, "Dispatched background heartbeat HELLO");
    }
}

esp_err_t discovery_node_init(void)
{
    s_node_announced = false;
    ensure_peer_exists(s_broadcast_mac);

    if (xTaskCreate(discovery_node_task, "disc_node_task", 3072, NULL, 5, &s_node_discovery_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create node discovery task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool discovery_node_is_announced(void)
{
    return s_node_announced;
}

bool discovery_node_handle_rx(const uint8_t *src_mac, const espnow_frame_t *frame, int16_t lqi)
{
    if (!src_mac || !frame) return false;

    // Layer 2: Respond to Coordinator DISCOVER_ALL sweep
    if (strcmp(frame->cmd, DISCOVERY_CMD_SWEEP) == 0 ||
        strcmp(frame->text, DISCOVERY_CMD_SWEEP) == 0) {

        ESP_LOGI(TAG, "Received DISCOVER_ALL sweep from Coordinator (%02X:%02X:%02X:%02X:%02X:%02X)",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);

        nvs_config_set_coord_mac(src_mac);
        ensure_peer_exists(src_mac);

        // Immediate unicast HELLO response
        espnow_frame_t reply = {0};
        strncpy(reply.source, nvs_config_get_name(), sizeof(reply.source) - 1);
        strncpy(reply.target, "coordinator", sizeof(reply.target) - 1);
        strncpy(reply.cmd, DISCOVERY_CMD_HELLO, sizeof(reply.cmd) - 1);
        strncpy(reply.text, DISCOVERY_CMD_HELLO, sizeof(reply.text) - 1);

        esp_now_send(src_mac, (const uint8_t *)&reply, sizeof(reply));
        ESP_LOGI(TAG, "Replied to DISCOVER_ALL sweep with unicast HELLO");
        s_node_announced = true;
        return true;
    }

    // Layer 1: Confirmation of node boot announcement
    if (strcmp(frame->cmd, DISCOVERY_CMD_HELLO_ACK) == 0 ||
        strcmp(frame->text, DISCOVERY_CMD_HELLO_ACK) == 0) {

        ESP_LOGI(TAG, "Received HELLO_ACK from Coordinator (%02X:%02X:%02X:%02X:%02X:%02X, LQI=%d)",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5], lqi);

        nvs_config_set_coord_mac(src_mac);
        ensure_peer_exists(src_mac);
        s_node_announced = true;
        return true;
    }

    // Automatically capture Coordinator MAC from any inbound traffic from coordinator
    if (strcasecmp(frame->source, "coordinator") == 0) {
        nvs_config_set_coord_mac(src_mac);
        ensure_peer_exists(src_mac);
    }

    return false;
}

