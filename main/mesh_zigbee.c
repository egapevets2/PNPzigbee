#include "mesh_zigbee.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_mac.h"

static const char *TAG = "MESH_ESPNOW";

// Seeed Studio XIAO ESP32-C6 RF Switch Control
// GPIO 3: RF switch enable (Active LOW - driving LOW powers ON the RF switch)
// GPIO 14: Antenna select (LOW = Onboard ceramic antenna, HIGH = External U.FL)
#define XIAO_RF_SWITCH_PWR_GPIO  GPIO_NUM_3
#define XIAO_RF_SWITCH_SEL_GPIO  GPIO_NUM_14

void mesh_zigbee_set_rf_pins(int pwr, int sel)
{
    gpio_reset_pin(XIAO_RF_SWITCH_PWR_GPIO);
    gpio_set_direction(XIAO_RF_SWITCH_PWR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XIAO_RF_SWITCH_PWR_GPIO, pwr);

    gpio_reset_pin(XIAO_RF_SWITCH_SEL_GPIO);
    gpio_set_direction(XIAO_RF_SWITCH_SEL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XIAO_RF_SWITCH_SEL_GPIO, sel);

    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "RF Switch set: PWR(GPIO3)=%d, SEL(GPIO14)=%d", pwr, sel);
}

void mesh_zigbee_set_rf_antenna(bool external)
{
    mesh_zigbee_set_rf_pins(0, external ? 1 : 0);
}

static void xiao_rf_switch_init(void)
{
    mesh_zigbee_set_rf_pins(0, 0); // PWR=0 (ON), SEL=0 (Ceramic)
}

static void wifi_promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    if (pkt && pkt->rx_ctrl.sig_len > 24) {
        ESP_LOGI("SNIFF", "PKT len=%d rssi=%d", pkt->rx_ctrl.sig_len, pkt->rx_ctrl.rssi);
    }
}

void mesh_zigbee_set_sniffer(bool enable)
{
    if (enable) {
        esp_wifi_set_promiscuous_rx_cb(wifi_promisc_cb);
        esp_wifi_set_promiscuous(true);
        ESP_LOGI(TAG, "Wi-Fi promiscuous sniffer ENABLED");
    } else {
        esp_wifi_set_promiscuous(false);
        ESP_LOGI(TAG, "Wi-Fi promiscuous sniffer DISABLED");
    }
}

#define ESPNOW_WIFI_CHANNEL 1
#define MESH_RX_QUEUE_LEN 16
#define MESH_TX_QUEUE_LEN 16

typedef struct __attribute__((packed))
{
    char source[MESH_TEXT_LEN];
    char target[MESH_TEXT_LEN];
    char cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2;
    char text[MESH_LINE_LEN];
} espnow_frame_t;

static QueueHandle_t s_mesh_rx_queue = NULL;
static QueueHandle_t s_mesh_tx_queue = NULL;
static int16_t s_last_rx_lqi = 255;
static bool s_joined = true;

// Known coordinator MAC address: B4:3A:45:8A:C7:18
static uint8_t s_coordinator_mac[ESP_NOW_ETH_ALEN] = {0xB4, 0x3A, 0x45, 0x8A, 0xC7, 0x18};
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool s_coordinator_known = true;
static char s_local_node_name[MESH_TEXT_LEN] = "Kitchen";

static void init_local_node_name(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        // Kitchen: B4:3A:45:8A:C6:40
        // Garage:  B4:3A:45:8A:C7:C0
        // Santafe: 58:E6:C5:1A:E8:A0
        // aa:      58:E6:C5:1A:DD:D0
        // bb:      58:E6:C5:13:6E:EC
        if (mac[4] == 0xC6 && mac[5] == 0x40) {
            strncpy(s_local_node_name, "Kitchen", sizeof(s_local_node_name) - 1);
        } else if (mac[4] == 0xC7 && mac[5] == 0xC0) {
            strncpy(s_local_node_name, "Garage", sizeof(s_local_node_name) - 1);
        } else if (mac[4] == 0xE8 && mac[5] == 0xA0) {
            strncpy(s_local_node_name, "Santafe", sizeof(s_local_node_name) - 1);
        } else if (mac[4] == 0xDD && mac[5] == 0xD0) {
            strncpy(s_local_node_name, "aa", sizeof(s_local_node_name) - 1);
        } else if (mac[4] == 0x6E && mac[5] == 0xEC) {
            strncpy(s_local_node_name, "bb", sizeof(s_local_node_name) - 1);
        } else {
            // Default to Garage if MAC doesn't match Kitchen
            strncpy(s_local_node_name, "Garage", sizeof(s_local_node_name) - 1);
        }
        ESP_LOGI(TAG, "Local node MAC: %02X:%02X:%02X:%02X:%02X:%02X -> identified as '%s'",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], s_local_node_name);
    }
}

static inline int16_t rssi_to_lqi(int8_t rssi)
{
    if (rssi <= -100) return 0;
    if (rssi >= -30)  return 255;
    return (int16_t)(((rssi + 100) * 255) / 70);
}

static bool ensure_peer_exists(const uint8_t *mac)
{
    if (!mac) return false;
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add peer %02x:%02x:%02x:%02x:%02x:%02x: %s",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], esp_err_to_name(err));
        return false;
    }

    esp_now_rate_config_t rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate = WIFI_PHY_RATE_LORA_250K,
        .ersu = false,
        .dcm = false,
    };
    esp_now_set_peer_rate_config(mac, &rate_cfg);
    return true;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (!recv_info || !data || len <= 0) return;

    ESP_LOGI(TAG, "RAW RX: from %02x:%02x:%02x:%02x:%02x:%02x len=%d (RSSI=%d)",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
             len, recv_info->rx_ctrl ? recv_info->rx_ctrl->rssi : 0);

    // Automatically update coordinator MAC from sender
    memcpy(s_coordinator_mac, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    ensure_peer_exists(s_coordinator_mac);
    s_coordinator_known = true;

    if (recv_info->rx_ctrl) {
        s_last_rx_lqi = rssi_to_lqi(recv_info->rx_ctrl->rssi);
    }

    mesh_msg_t msg = {0};

    if (len == sizeof(espnow_frame_t)) {
        const espnow_frame_t *frame = (const espnow_frame_t *)data;

        // Verify target filtering against dynamic local node name
        if (frame->target[0] != '\0' &&
            strcasecmp(frame->target, s_local_node_name) != 0 &&
            strcasecmp(frame->target, "broadcast") != 0) {
            ESP_LOGD(TAG, "Ignoring frame addressed to '%s'", frame->target);
            return; // Not addressed to this node
        }

        strncpy(msg.source, frame->source[0] ? frame->source : "coordinator", sizeof(msg.source) - 1);
        strncpy(msg.target, frame->target, sizeof(msg.target) - 1);
        strncpy(msg.cmd, frame->cmd, sizeof(msg.cmd) - 1);
        msg.value = frame->value;
        msg.value2 = frame->value2;
        strncpy(msg.text, frame->text, sizeof(msg.text) - 1);
        ESP_LOGI(TAG, "ESP-NOW RX frame: src='%s' tgt='%s' cmd='%s' text='%s'",
                 msg.source, msg.target, msg.cmd, msg.text);
    } else {
        // Plain ASCII string fallback
        size_t copy_len = (size_t)len < (sizeof(msg.text) - 1) ? (size_t)len : (sizeof(msg.text) - 1);
        memcpy(msg.text, data, copy_len);
        msg.text[copy_len] = '\0';
        strncpy(msg.source, "coordinator", sizeof(msg.source) - 1);
        ESP_LOGI(TAG, "ESP-NOW RX text: src='%s' text='%s'", msg.source, msg.text);
    }

    if (s_mesh_rx_queue) {
        if (xQueueSend(s_mesh_rx_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX queue full, dropping packet: %s", msg.text);
        }
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "ESP-NOW TX cb: status=%d (%s)", status, status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void mesh_tx_task(void *arg)
{
    mesh_msg_t msg;
    while (1) {
        if (xQueueReceive(s_mesh_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.source[0] == '\0') {
                strncpy(msg.source, s_local_node_name, sizeof(msg.source) - 1);
            }

            espnow_frame_t frame = {0};
            strncpy(frame.source, msg.source, sizeof(frame.source) - 1);
            strncpy(frame.target, msg.target[0] ? msg.target : "coordinator", sizeof(frame.target) - 1);
            strncpy(frame.cmd, msg.cmd, sizeof(frame.cmd) - 1);
            frame.value = msg.value;
            frame.value2 = msg.value2;
            strncpy(frame.text, msg.text, sizeof(frame.text) - 1);

            const uint8_t *dest_mac = (strcasecmp(frame.target, "broadcast") == 0) ? s_broadcast_mac :
                                      (s_coordinator_known ? s_coordinator_mac : s_broadcast_mac);
            ensure_peer_exists(dest_mac);

            ESP_LOGI(TAG, "TX task: sending to %02x:%02x:%02x:%02x:%02x:%02x text='%s'",
                     dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5],
                     frame.text);

            esp_err_t err = esp_now_send(dest_mac, (const uint8_t *)&frame, sizeof(frame));
            ESP_LOGI(TAG, "esp_now_send returned %s (%d)", esp_err_to_name(err), err);

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_now_send to unicast failed: %s, falling back to broadcast", esp_err_to_name(err));
                ensure_peer_exists(s_broadcast_mac);
                esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

esp_err_t mesh_zigbee_init(void)
{
    if (!s_mesh_rx_queue) {
        s_mesh_rx_queue = xQueueCreate(MESH_RX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    if (!s_mesh_tx_queue) {
        s_mesh_tx_queue = xQueueCreate(MESH_TX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    ESP_RETURN_ON_FALSE(s_mesh_rx_queue && s_mesh_tx_queue, ESP_FAIL, TAG, "Failed to create queues");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS flash init failed");

    esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK && netif_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(netif_err, TAG, "esp_netif_init failed");
    }

    esp_err_t evt_err = esp_event_loop_create_default();
    if (evt_err != ESP_OK && evt_err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(evt_err, TAG, "esp_event_loop_create_default failed");
    }

    // Power on Seeed Studio XIAO ESP32-C6 RF switch and select ceramic antenna
    xiao_rf_switch_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); // 80 * 0.25 dBm = 20 dBm (Maximum RF TX Power)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Re-assert RF switch after Wi-Fi start in case Wi-Fi init affected pin configuration
    xiao_rf_switch_init();

    uint8_t primary_chan = 0;
    wifi_second_chan_t second_chan = 0;
    esp_wifi_get_channel(&primary_chan, &second_chan);
    int8_t max_tx_power = 0;
    esp_wifi_get_max_tx_power(&max_tx_power);
    ESP_LOGI(TAG, "Wi-Fi Config: Actual Channel=%d (second=%d), Max TX Power=%d (0.25dBm units, %.2fdBm), LR Mode enabled",
             primary_chan, second_chan, max_tx_power, max_tx_power * 0.25f);
    ESP_LOGI(TAG, "RF Switch pins: PWR(GPIO3)=%d, SEL(GPIO14)=%d",
             gpio_get_level(XIAO_RF_SWITCH_PWR_GPIO), gpio_get_level(XIAO_RF_SWITCH_SEL_GPIO));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    ensure_peer_exists(s_broadcast_mac);
    ensure_peer_exists(s_coordinator_mac);

    init_local_node_name();

    s_joined = true;
    ESP_LOGI(TAG, "ESP-NOW radio initialized on Channel %d (%s node online)", primary_chan, s_local_node_name);

    xTaskCreate(mesh_tx_task, "mesh_tx_task", 3072, NULL, 5, NULL);

    return ESP_OK;
}

bool mesh_zigbee_send(mesh_msg_t msg)
{
    if (!s_mesh_tx_queue) return false;
    return (xQueueSend(s_mesh_tx_queue, &msg, 0) == pdTRUE);
}

bool mesh_zigbee_send_text(const char *text)
{
    if (!text || text[0] == '\0') return false;
    mesh_msg_t msg = {0};
    strncpy(msg.source, s_local_node_name, sizeof(msg.source) - 1);
    strncpy(msg.target, "coordinator", sizeof(msg.target) - 1);
    strncpy(msg.text, text, sizeof(msg.text) - 1);
    return mesh_zigbee_send(msg);
}

bool mesh_zigbee_send_cmd(const char *cmd, int16_t value)
{
    if (!cmd || cmd[0] == '\0') return false;
    mesh_msg_t msg = {0};
    strncpy(msg.source, s_local_node_name, sizeof(msg.source) - 1);
    strncpy(msg.target, "coordinator", sizeof(msg.target) - 1);
    strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1);
    msg.value = value;
    return mesh_zigbee_send(msg);
}

bool mesh_zigbee_receive(mesh_msg_t *msg, TickType_t timeout)
{
    if (!s_mesh_rx_queue || !msg) return false;
    return (xQueueReceive(s_mesh_rx_queue, msg, timeout) == pdTRUE);
}

bool mesh_zigbee_is_joined(void)
{
    return s_joined;
}

bool mesh_zigbee_send_broadcast(const char *text)
{
    if (!text || text[0] == '\0') return false;
    mesh_msg_t msg = {0};
    strncpy(msg.source, s_local_node_name, sizeof(msg.source) - 1);
    strncpy(msg.target, "broadcast", sizeof(msg.target) - 1);
    strncpy(msg.text, text, sizeof(msg.text) - 1);
    return mesh_zigbee_send(msg);
}

int16_t mesh_zigbee_last_lqi(void)
{
    return s_last_rx_lqi;
}

