/*
 * serial_gateway.c - Coordinator Serial Gateway & Central Host Interface
 *
 * Purpose:
 *   Operates as the central USB serial gateway on the Coordinator node, interfacing between
 *   the PC host (e.g. test runner, command terminal) and the remote mesh network.
 *   - Routes commands from PC host to specific named end devices (e.g. "Kitchen blink 3").
 *   - Receives upstream replies and notifications from end devices and streams them to host.
 *   - Coordinates network topology queries, discovery sweeps, and link-quality reports.
 *
 * Supported Commands (Coordinator Host Interface):
 *   <TargetDevice> <cmd> [args...]
 *     - Routes the command line directly to the named destination end device (e.g. "Kitchen blink 3").
 *
 *   GiveNetworkReport
 *     Initiates an active network survey: unicasts PING frames to all registered nodes in the
 *     routing table, computes real-time RSSI/LQI round-trips, and outputs a formatted topology summary.
 *
 *   PingNetwork
 *     Unicasts PING to all active nodes in the routing table to verify bidirectional connectivity.
 *
 *   ResetNetwork
 *     Clears the coordinator routing table and broadcasts a Layer 2 DISCOVER_ALL sweep to rebuild topology.
 *
 *   ResetCoordinator
 *     Reboots the coordinator hardware.
 *
 *   scan
 *     Triggers a local 2.4GHz Wi-Fi environment scan and prints all detected access points.
 *
 *   rf <pwr> <sel>
 *     Directly sets the coordinator hardware RF switch pins (GPIO 3 PWR, GPIO 14 SEL).
 *
 *   help / ?
 *     Prints available coordinator host commands.
 */

#include "serial_gateway.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "mesh_serial.h"
#include "mesh_zigbee.h"
#include "nvs_config.h"
#include "discovery.h"

static const char *TAG = "SERIAL_GATEWAY";

typedef routing_entry_t DeviceEntry;
#define DEVICE_COUNT (routing_table_get_count())

#define find_device_by_name(name) routing_table_find_by_name(name)
#define find_device_by_mac(mac)   routing_table_find_by_mac(mac)

#define COORD_RX_QUEUE_LEN 16
#define COORD_TX_QUEUE_LEN 16

static QueueHandle_t s_coord_rx_queue = NULL;
static QueueHandle_t s_coord_tx_queue = NULL;

#define FRESH_NETWORK_REPORT_TIMEOUT_MS 3000
static volatile bool s_fresh_report_active = false;
static volatile int s_fresh_report_expected = 0;
static volatile int s_fresh_report_received = 0;

static void strip_line_endings(char *s)
{
    if (!s) return;
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n')) {
        s[len - 1] = '\0';
        len--;
    }
}

static esp_err_t coord_init_queues(void)
{
    if (!s_coord_rx_queue) {
        s_coord_rx_queue = xQueueCreate(COORD_RX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    if (!s_coord_tx_queue) {
        s_coord_tx_queue = xQueueCreate(COORD_TX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    ESP_RETURN_ON_FALSE(s_coord_rx_queue && s_coord_tx_queue, ESP_FAIL, TAG, "Failed to create coordinator queues");
    return ESP_OK;
}

static bool coord_enqueue_outgoing(mesh_msg_t msg)
{
    if (!s_coord_tx_queue) return false;
    return (xQueueSend(s_coord_tx_queue, &msg, 0) == pdTRUE);
}

static bool coord_dequeue_incoming(mesh_msg_t *msg, TickType_t timeout)
{
    if (!s_coord_rx_queue || !msg) return false;
    return (xQueueReceive(s_coord_rx_queue, msg, timeout) == pdTRUE);
}

static void parse_console_line_to_mesh_msg(const char *line, mesh_msg_t *msg)
{
    if (!line || !msg) return;
    memset(msg, 0, sizeof(*msg));
    strncpy(msg->text, line, sizeof(msg->text) - 1);

    char target[MESH_TEXT_LEN] = {0};
    char cmd[MESH_TEXT_LEN] = {0};
    int value = 0;
    int value2 = 0;

    int fields = sscanf(line, "%17s %17s %d %d", target, cmd, &value, &value2);
    if (fields >= 1) {
        strncpy(msg->target, target, sizeof(msg->target) - 1);
    }
    if (fields >= 2) {
        strncpy(msg->cmd, cmd, sizeof(msg->cmd) - 1);
    }
    if (fields >= 3) {
        if (value > INT16_MAX) {
            msg->value = INT16_MAX;
        } else if (value < INT16_MIN) {
            msg->value = INT16_MIN;
        } else {
            msg->value = (int16_t)value;
        }
    }
    if (fields >= 4) {
        if (value2 > INT16_MAX) {
            msg->value2 = INT16_MAX;
        } else if (value2 < INT16_MIN) {
            msg->value2 = INT16_MIN;
        } else {
            msg->value2 = (int16_t)value2;
        }
    }
}

static const char *device_route_string(const DeviceEntry *dev)
{
    return (dev && dev->online && dev->short_addr != 0xFFFF) ? "YES" : "NO";
}

static const char *device_rf_status_string(const DeviceEntry *dev)
{
    if (!dev || !dev->online || dev->short_addr == 0xFFFF) {
        return "UNKNOWN";
    }
    int weakest_lqi = dev->last_lqi;
    if (dev->remote_lqi >= 0 && dev->remote_lqi < weakest_lqi) {
        weakest_lqi = dev->remote_lqi;
    }
    if (weakest_lqi < 50) return "WEAK";
    if (weakest_lqi < 100) return "MARGINAL";
    return "OK";
}

static void print_network_report(void)
{
    mesh_serial_write("\r\n..... recieved GiveNetworkReport --------\r\n\r\n");
    mesh_serial_write("*********************NETWORK_REPORT_BEGIN\r\n\r\n");
    mesh_serial_write("NAME,IEEE,SHORT,ONLINE,TYPE,LQI,REMOTE_LQI,RF_STATUS,AGE,ROUTE\r\n");

    uint32_t now = xTaskGetTickCount();

    size_t count = routing_table_get_count();
    for (size_t i = 0; i < count; i++) {
        DeviceEntry *dev = routing_table_get_by_index(i);
        if (!dev) continue;
        uint32_t age_sec = 0;
        if (dev->last_seen_tick != 0) {
            age_sec = (now - dev->last_seen_tick) / configTICK_RATE_HZ;
        }

        const char *type = dev->online ? "END_DEVICE" : "UNKNOWN";
        const char *route = device_route_string(dev);
        const char *rf_status = device_rf_status_string(dev);

        char remote_lqi_str[8];
        if (dev->remote_lqi >= 0) {
            snprintf(remote_lqi_str, sizeof(remote_lqi_str), "%d", dev->remote_lqi);
        } else {
            snprintf(remote_lqi_str, sizeof(remote_lqi_str), "-");
        }

        mesh_serial_writef(
            "%s,0x%016llX,0x%04X,%d,%s,%d,%s,%s,%lu,%s\r\n",
            dev->name,
            (unsigned long long)dev->ieee,
            dev->short_addr,
            dev->online ? 1 : 0,
            type,
            dev->last_lqi,
            remote_lqi_str,
            rf_status,
            (unsigned long)age_sec,
            route);
    }

    mesh_serial_write("*********************NETWORK_REPORT_END\r\n\r\n");
    mesh_serial_write("      ----------------------\r\n");
    mesh_serial_write("      |   RF_STATUS_LEGEND  |\r\n");
    mesh_serial_write("      ----------------------\r\n");
    mesh_serial_write("MIN_LQI,MAX_LQI,RF_STATUS\r\n");
    mesh_serial_write("0,49,WEAK\r\n");
    mesh_serial_write("50,99,MARGINAL\r\n");
    mesh_serial_write("100,255,OK\r\n");
    mesh_serial_write("offline,offline,UNKNOWN\r\n\r\n");
    mesh_serial_write("RF_STATUS_LEGEND_END\r\n");
}

static void ping_network(void)
{
    mesh_serial_write("PING_NETWORK_BEGIN\r\n");
    size_t count = routing_table_get_count();
    for (size_t i = 0; i < count; i++) {
        DeviceEntry *dev = routing_table_get_by_index(i);
        if (!dev || dev->mac[0] == 0) continue;

        mesh_msg_t msg = {0};
        strncpy(msg.target, dev->name, sizeof(msg.target) - 1);
        strncpy(msg.text, "PING", sizeof(msg.text) - 1);

        if (coord_enqueue_outgoing(msg)) {
            mesh_serial_writef("PING %s SENT SHORT=0x%04X\r\n", dev->name, dev->short_addr);
            ESP_LOGI(TAG, "PingNetwork: queued PING to %s", dev->name);
        } else {
            mesh_serial_writef("PING %s ERROR=TX_QUEUE_FULL\r\n", dev->name);
            ESP_LOGW(TAG, "PingNetwork: TX queue full for %s", dev->name);
        }
    }
    mesh_serial_write("PING_NETWORK_END\r\n");
}

static void fresh_network_report_task(void *arg)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(FRESH_NETWORK_REPORT_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        if (s_fresh_report_expected <= 0 || s_fresh_report_received >= s_fresh_report_expected) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    mesh_serial_writef("FRESH_REPORT_RESPONSES RECEIVED=%d EXPECTED=%d\r\n",
                       s_fresh_report_received,
                       s_fresh_report_expected);

    size_t count = routing_table_get_count();
    for (size_t i = 0; i < count; i++) {
        DeviceEntry *dev = routing_table_get_by_index(i);
        if (!dev) continue;
        if (dev->fresh_ping_expected && !dev->fresh_ping_seen) {
            dev->online = false;
            dev->last_lqi = 0;
            dev->remote_lqi = -1;
            ESP_LOGW(TAG, "Fresh report: no PONG from %s, marking offline", dev->name);
        }
        dev->fresh_ping_expected = false;
        dev->fresh_ping_seen = false;
    }

    print_network_report();
    s_fresh_report_active = false;
    vTaskDelete(NULL);
}

static void start_fresh_network_report(void)
{
    if (s_fresh_report_active) {
        mesh_serial_write("error: fresh network report already running\r\n");
        return;
    }

    if (routing_table_get_count() == 0) {
        discovery_coord_sweep();
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    size_t count = routing_table_get_count();
    int expected = 0;
    for (size_t i = 0; i < count; i++) {
        DeviceEntry *dev = routing_table_get_by_index(i);
        if (!dev) continue;
        dev->fresh_ping_expected = true;
        dev->fresh_ping_seen = false;
        expected++;
    }

    s_fresh_report_expected = expected;
    s_fresh_report_received = 0;
    s_fresh_report_active = true;

    mesh_serial_write("FRESH_NETWORK_REPORT_BEGIN\r\n");
    ping_network();

    if (xTaskCreate(fresh_network_report_task, "fresh_report_task", 3072, NULL, 4, NULL) != pdPASS) {
        mesh_serial_write("error: failed to start fresh report task\r\n");
        s_fresh_report_active = false;
    }
}

static void reset_network_discovery(void)
{
    mesh_serial_write("RESET_NETWORK_BEGIN\r\n");
    routing_table_clear();
    discovery_coord_sweep();
    mesh_serial_write("RESET_NETWORK_END\r\n");
}

static void reset_coordinator(void)
{
    mesh_serial_write("RESET_COORDINATOR_BEGIN\r\n");
    mesh_serial_write("RESET_COORDINATOR_RESTARTING\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void coord_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (!recv_info || !data || len <= 0) return;

    ESP_LOGI(TAG, "ESP-NOW RX: from %02x:%02x:%02x:%02x:%02x:%02x len=%d",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5], len);

    int16_t lqi = 0;
    if (recv_info->rx_ctrl) {
        lqi = rssi_to_lqi(recv_info->rx_ctrl->rssi);
    }

    if (len == sizeof(espnow_frame_t)) {
        const espnow_frame_t *frame = (const espnow_frame_t *)data;

        // Tri-Layer Discovery protocol check
        if (discovery_coord_handle_rx(recv_info->src_addr, frame, lqi)) {
            return;
        }
    }

    DeviceEntry *dev = find_device_by_mac(recv_info->src_addr);
    if (dev) {
        dev->online = true;
        dev->last_seen_tick = xTaskGetTickCount();
        dev->last_lqi = lqi;
        ESP_LOGI(TAG, "Device %s ONLINE (LQI=%d)", dev->name, lqi);
    }

    mesh_msg_t rx = {0};

    if (len == sizeof(espnow_frame_t)) {
        const espnow_frame_t *frame = (const espnow_frame_t *)data;
        if (dev) {
            strncpy(rx.source, dev->name, sizeof(rx.source) - 1);
        } else if (frame->source[0] != '\0') {
            strncpy(rx.source, frame->source, sizeof(rx.source) - 1);
        } else {
            strncpy(rx.source, "unknown", sizeof(rx.source) - 1);
        }
        strncpy(rx.target, frame->target, sizeof(rx.target) - 1);
        strncpy(rx.cmd, frame->cmd, sizeof(rx.cmd) - 1);
        rx.value = frame->value;
        rx.value2 = frame->value2;
        strncpy(rx.text, frame->text, sizeof(rx.text) - 1);
        ESP_LOGI(TAG, "ESP-NOW RX frame: src='%s' tgt='%s' cmd='%s' text='%s'",
                 rx.source, rx.target, rx.cmd, rx.text);
    } else {
        size_t copy_len = (size_t)len < (sizeof(rx.text) - 1) ? (size_t)len : (sizeof(rx.text) - 1);
        memcpy(rx.text, data, copy_len);
        rx.text[copy_len] = '\0';
        if (dev) {
            strncpy(rx.source, dev->name, sizeof(rx.source) - 1);
        } else {
            strncpy(rx.source, "unknown", sizeof(rx.source) - 1);
        }
        ESP_LOGI(TAG, "ESP-NOW RX text: src='%s' text='%s'", rx.source, rx.text);
    }

    rx.value = lqi;

    if (strncmp(rx.text, "PONG", 4) == 0 && dev) {
        int remote_lqi = -1;
        char *p = strstr(rx.text, "LQI=");
        if (p) {
            remote_lqi = atoi(p + 4);
        }
        dev->remote_lqi = remote_lqi;
        dev->fresh_ping_seen = true;
    }

    if (s_coord_rx_queue) {
        if (xQueueSend(s_coord_rx_queue, &rx, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX queue full, dropping text from %s", rx.source);
        }
    }
}

static void coord_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "ESP-NOW TX cb: status=%d (%s)", status, status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void coord_serial_console_task(void *arg)
{
    char line[MESH_LINE_LEN] = {0};
    size_t pos = 0;
    bool last_was_cr = false;

    ESP_LOGI(TAG, "PuTTY bridge interface ready: <target> <text>");
    mesh_serial_write("\r\n--- ESP-NOW Coordinator Bridge ---\r\n");
    mesh_serial_write("\r\nCoordinator bridge ready. Use: <DeviceName> <text>\r\n");
    mesh_serial_write("Examples: Kitchen blink 3  |  Garage ChickenSoup88\r\n");
    mesh_serial_write("Commands: GiveNetworkReport, PingNetwork, ResetNetwork, ResetCoordinator\r\n\r\ncoordinator> ");

    while (1) {
        int ch = mesh_serial_read_byte();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\n' && last_was_cr) {
            last_was_cr = false;
            continue;
        }
        last_was_cr = (ch == '\r');

        if (ch == '\r' || ch == '\n') {
            if (pos == 0) {
                mesh_serial_write("\r\ncoordinator> ");
                continue;
            }

            line[pos] = '\0';
            strip_line_endings(line);
            mesh_serial_writef("\r\n> %s\r\n", line);

            if (nvs_config_handle_cli(line)) {
                pos = 0;
                memset(line, 0, sizeof(line));
                mesh_serial_write("coordinator> ");
                continue;
            }

            if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
                mesh_serial_write("\r\nCommands: GiveNetworkReport, PingNetwork, ResetNetwork, ResetCoordinator, scan, rf <pwr> <sel>\r\n");
                mesh_serial_write("Usage: <DeviceName> <command/text> (e.g. Kitchen blink 3)\r\ncoordinator> ");
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strncmp(line, "rf ", 3) == 0) {
                int pwr = 0, sel = 0;
                if (sscanf(line + 3, "%d %d", &pwr, &sel) == 2) {
                    mesh_zigbee_set_rf_pins(pwr, sel);
                    mesh_serial_writef("RF pins set: PWR=%d, SEL=%d\r\n", pwr, sel);
                }
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "scan") == 0) {
                mesh_serial_write("Scanning Wi-Fi...\r\n");
                wifi_scan_config_t scan_cfg = {0};
                esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
                if (err == ESP_OK) {
                    uint16_t ap_count = 0;
                    esp_wifi_scan_get_ap_num(&ap_count);
                    mesh_serial_writef("Found %d APs\r\n", ap_count);
                    if (ap_count > 0) {
                        wifi_ap_record_t *ap_list = malloc(sizeof(wifi_ap_record_t) * ap_count);
                        if (ap_list) {
                            esp_wifi_scan_get_ap_records(&ap_count, ap_list);
                            for (int i = 0; i < ap_count && i < 10; i++) {
                                mesh_serial_writef("  SSID: %s, RSSI: %d, Chan: %d\r\n",
                                                   ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);
                            }
                            free(ap_list);
                        }
                    }
                } else {
                    mesh_serial_writef("Scan failed: %s\r\n", esp_err_to_name(err));
                }
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "GiveNetworkReport") == 0) {
                start_fresh_network_report();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "PingNetwork") == 0) {
                ping_network();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "ResetNetwork") == 0) {
                reset_network_discovery();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "ResetCoordinator") == 0) {
                reset_coordinator();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            mesh_msg_t msg = {0};
            parse_console_line_to_mesh_msg(line, &msg);

            if (msg.target[0] == '\0') {
                mesh_serial_write("error: missing target\r\n");
                ESP_LOGW(TAG, "Serial line ignored: missing target");
            } else if (!coord_enqueue_outgoing(msg)) {
                mesh_serial_write("error: TX queue full\r\n");
                ESP_LOGW(TAG, "TX queue full, dropping serial line: %s", line);
            } else {
                mesh_serial_write("queued\r\n");
                ESP_LOGI(TAG, "SERIAL TX queued: %s", line);
            }

            pos = 0;
            memset(line, 0, sizeof(line));
        }
        else if (ch == 8 || ch == 127) {
            if (pos > 0) {
                pos--;
                line[pos] = '\0';
                mesh_serial_write("\b \b");
            }
        }
        else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)ch;
            char echo[2] = {(char)ch, 0};
            mesh_serial_write(echo);
        }
        else {
            mesh_serial_write("\r\nerror: line too long\r\n");
            pos = 0;
            memset(line, 0, sizeof(line));
        }
    }
}

static void coord_mesh_tx_task(void *arg)
{
    mesh_msg_t msg;

    while (1) {
        if (xQueueReceive(s_coord_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.target[0] == '\0') continue;

            DeviceEntry *dev = find_device_by_name(msg.target);
            const uint8_t *dest_mac = dev ? dev->mac : s_broadcast_mac;
            ensure_peer_exists(dest_mac);

            espnow_frame_t frame = {0};
            strncpy(frame.source, "coordinator", sizeof(frame.source) - 1);
            strncpy(frame.target, msg.target, sizeof(frame.target) - 1);
            strncpy(frame.cmd, msg.cmd, sizeof(frame.cmd) - 1);
            frame.value = msg.value;
            frame.value2 = msg.value2;
            strncpy(frame.text, msg.text, sizeof(frame.text) - 1);

            ESP_LOGI(TAG, "TX task: sending to %02x:%02x:%02x:%02x:%02x:%02x target='%s' text='%s'",
                     dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5],
                     frame.target, frame.text);

            esp_err_t err = esp_now_send(dest_mac, (const uint8_t *)&frame, sizeof(frame));
            ESP_LOGI(TAG, "esp_now_send returned %s (%d)", esp_err_to_name(err), err);

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_now_send to %s failed: %s, trying broadcast", msg.target, esp_err_to_name(err));
                ensure_peer_exists(s_broadcast_mac);
                esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void coord_app_msg_task(void *arg)
{
    mesh_msg_t msg;

    while (1) {
        if (coord_dequeue_incoming(&msg, portMAX_DELAY)) {
            if (msg.text[0] != '\0') {
                ESP_LOGI(TAG, "SERIAL RX from %s: %s", msg.source, msg.text);
                if (strcmp(msg.text, "ACK") == 0) {
                    mesh_serial_writef("< %s: [ACK]\r\n", msg.source);
                }
                else if (strncmp(msg.text, "PONG", 4) == 0) {
                    int remote_lqi = -1;
                    char *p = strstr(msg.text, "LQI=");
                    if (p) {
                        remote_lqi = atoi(p + 4);
                    }
                    mesh_serial_writef("< %s: [PONG LQI=%d REMOTE_LQI=%d]\r\n",
                                       msg.source,
                                       msg.value,
                                       remote_lqi);
                    if (s_fresh_report_active) {
                        s_fresh_report_received++;
                    }
                }
                else {
                    mesh_serial_writef("< %s: %s\r\n", msg.source, msg.text);
                }

                if (strcmp(msg.text, "ACK") != 0 &&
                    strncmp(msg.text, "PONG", 4) != 0 &&
                    msg.source[0] != '\0') {
                    mesh_msg_t ack = {0};
                    strncpy(ack.target, msg.source, sizeof(ack.target) - 1);
                    strncpy(ack.text, "ACK", sizeof(ack.text) - 1);
                    coord_enqueue_outgoing(ack);
                }
            }
            else if (strcmp(msg.cmd, "button") == 0) {
                ESP_LOGI(TAG, "%s Button Pressed", msg.source);
                mesh_serial_writef("< %s: button %d\r\n", msg.source, msg.value);
            }
            else {
                ESP_LOGI(TAG, "APP RX source='%s' cmd='%s' value=%d",
                         msg.source, msg.cmd, msg.value);
                mesh_serial_writef("< %s: %s %d\r\n", msg.source, msg.cmd, msg.value);
            }
        }
    }
}

esp_err_t serial_gateway_init(void)
{
    ESP_RETURN_ON_ERROR(mesh_radio_init(), TAG, "Failed to initialize mesh radio");
    ESP_RETURN_ON_ERROR(coord_init_queues(), TAG, "Failed to create coordinator queues");

    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(coord_espnow_recv_cb), TAG, "Failed to register recv cb");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(coord_espnow_send_cb), TAG, "Failed to register send cb");

    ESP_RETURN_ON_ERROR(discovery_coord_init(), TAG, "Failed to initialize coordinator discovery");

    ESP_LOGI(TAG, "ESP-NOW Coordinator gateway initialized on Channel %d", nvs_config_get_channel());

    xTaskCreate(coord_mesh_tx_task, "coord_mesh_tx", 3072, NULL, 5, NULL);
    xTaskCreate(coord_app_msg_task, "coord_app_msg", 3072, NULL, 5, NULL);
    xTaskCreate(coord_serial_console_task, "coord_console", 3072, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(200));
    ping_network();

    return ESP_OK;
}

