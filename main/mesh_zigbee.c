#include "mesh_zigbee.h"

#include <stdio.h>
#include <string.h>

#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "esp_zb_light.h"
#include "aps/esp_zigbee_aps.h"

#if !defined(ZB_ED_ROLE) && !defined(ZB_ROUTER_ROLE)
#error Define either ZB_ED_ROLE or ZB_ROUTER_ROLE in idf.py menuconfig to compile this light/router source code.
#endif

static const char *TAG = "MESH_ZIGBEE";

#define APS_MSG_CLUSTER_ID 0xFFC0
#define COORDINATOR_ENDPOINT 1
#define CMD_LEN 18
#define MESH_RX_QUEUE_LEN 16
#define MESH_TX_QUEUE_LEN 16

typedef struct __attribute__((packed))
{
    int16_t value;
    char cmd[CMD_LEN];
} AppMessage;

static QueueHandle_t s_mesh_rx_queue = NULL;
static QueueHandle_t s_mesh_tx_queue = NULL;
static SemaphoreHandle_t s_aps_tx_sem = NULL;
static bool s_joined = false;
static int16_t s_last_rx_lqi = 0;
static uint8_t s_steering_retry_count = 0;

static void esp_zb_task(void *pvParameters);
static void mesh_tx_task(void *arg);
static void send_aps_command_to_coordinator(const char *cmd, int16_t value);
static void send_aps_string_to_coordinator(const char *msg);

static bool enqueue_incoming_from_zigbee(mesh_msg_t msg)
{
    if (!s_mesh_rx_queue) {
        ESP_LOGW(TAG, "RX queue not initialized");
        return false;
    }
    return xQueueSend(s_mesh_rx_queue, &msg, 0) == pdTRUE;
}

static bool aps_payload_looks_like_text(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return false;
    }
    return (data[0] >= 0x20 && data[0] <= 0x7e);
}

esp_err_t mesh_zigbee_init(void)
{
    if (!s_mesh_rx_queue) {
        s_mesh_rx_queue = xQueueCreate(MESH_RX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    if (!s_mesh_tx_queue) {
        s_mesh_tx_queue = xQueueCreate(MESH_TX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    if (!s_aps_tx_sem) {
        s_aps_tx_sem = xSemaphoreCreateBinary();
    }
    ESP_RETURN_ON_FALSE(s_mesh_rx_queue && s_mesh_tx_queue && s_aps_tx_sem, ESP_FAIL, TAG, "Failed to create mesh queues/semaphores");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init failed");

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_RETURN_ON_ERROR(esp_zb_platform_config(&config), TAG, "esp_zb_platform_config failed");

    xTaskCreate(mesh_tx_task, "mesh_tx_task", 3072, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    return ESP_OK;
}

bool mesh_zigbee_send(mesh_msg_t msg)
{
    if (!s_mesh_tx_queue) {
        ESP_LOGW(TAG, "TX queue not initialized");
        return false;
    }
    return xQueueSend(s_mesh_tx_queue, &msg, 0) == pdTRUE;
}

bool mesh_zigbee_send_text(const char *text)
{
    if (!text || text[0] == '\0') {
        return false;
    }

    mesh_msg_t msg = {0};
    strncpy(msg.target, "coordinator", sizeof(msg.target) - 1);
    strncpy(msg.text, text, sizeof(msg.text) - 1);
    return mesh_zigbee_send(msg);
}

bool mesh_zigbee_send_cmd(const char *cmd, int16_t value)
{
    if (!cmd || cmd[0] == '\0') {
        return false;
    }

    mesh_msg_t msg = {0};
    strncpy(msg.target, "coordinator", sizeof(msg.target) - 1);
    strncpy(msg.cmd, cmd, sizeof(msg.cmd) - 1);
    msg.value = value;
    return mesh_zigbee_send(msg);
}

bool mesh_zigbee_receive(mesh_msg_t *msg, TickType_t timeout)
{
    if (!s_mesh_rx_queue || !msg) {
        return false;
    }
    return xQueueReceive(s_mesh_rx_queue, msg, timeout) == pdTRUE;
}

bool mesh_zigbee_is_joined(void)
{
    return s_joined;
}

int16_t mesh_zigbee_last_lqi(void)
{
    return s_last_rx_lqi;
}

static void send_aps_command_to_coordinator(const char *cmd, int16_t value)
{
    static AppMessage msg;

    memset(&msg, 0, sizeof(msg));
    msg.value = value;
    strncpy(msg.cmd, cmd, CMD_LEN - 1);
    msg.cmd[CMD_LEN - 1] = '\0';

    esp_zb_apsde_data_req_t req = {
        .dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .dst_addr.addr_short = 0x0000,
        .dst_endpoint = COORDINATOR_ENDPOINT,
        .src_endpoint = HA_ESP_LIGHT_ENDPOINT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = APS_MSG_CLUSTER_ID,
        .tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX,
        .radius = 10,
        .asdu_length = sizeof(msg),
        .asdu = (uint8_t *)&msg,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_aps_data_request(&req);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "APS event send returned: %d", ret);
}

static void send_aps_string_to_coordinator(const char *msg)
{
    uint16_t len = strlen(msg) + 1;

    esp_zb_apsde_data_req_t req = {
        .dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .dst_addr.addr_short = 0x0000,
        .dst_endpoint = COORDINATOR_ENDPOINT,
        .src_endpoint = HA_ESP_LIGHT_ENDPOINT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = APS_MSG_CLUSTER_ID,
        .tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX,
        .radius = 10,
        .asdu_length = len,
        .asdu = (uint8_t *)msg,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_aps_data_request(&req);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "APS reply send returned: %d", ret);
}

static void zb_apsde_data_confirm_handler(esp_zb_apsde_data_confirm_t confirm)
{
    if (confirm.status == 0x00) {
        ESP_LOGI(TAG, "APS TX OK to 0x%04hx ep %d", confirm.dst_addr.addr_short, confirm.dst_endpoint);
    } else {
        ESP_LOGE(TAG, "APS TX FAIL status=%d", confirm.status);
    }
    if (s_aps_tx_sem) {
        xSemaphoreGive(s_aps_tx_sem);
    }
}

static bool zb_apsde_data_indication_handler(esp_zb_apsde_data_ind_t ind)
{
    if (ind.status == 0x00 &&
        ind.dst_endpoint == HA_ESP_LIGHT_ENDPOINT &&
        ind.profile_id == ESP_ZB_AF_HA_PROFILE_ID &&
        ind.cluster_id == APS_MSG_CLUSTER_ID) {

        s_last_rx_lqi = ind.lqi;

        if (ind.asdu_length != sizeof(AppMessage) ||
            aps_payload_looks_like_text((const uint8_t *)ind.asdu, ind.asdu_length)) {

            size_t len = ind.asdu_length;
            if (len >= MESH_LINE_LEN) {
                len = MESH_LINE_LEN - 1;
            }

            char text[MESH_LINE_LEN] = {0};
            memcpy(text, ind.asdu, len);
            text[len] = '\0';

            mesh_msg_t rx = {0};
            strncpy(rx.source, "coordinator", sizeof(rx.source) - 1);
            strncpy(rx.text, text, sizeof(rx.text) - 1);

            if (!enqueue_incoming_from_zigbee(rx)) {
                ESP_LOGW(TAG, "RX queue full, dropping text='%s'", text);
            }

            ESP_LOGI(TAG, "APS RX text queued from coordinator LQI=%d: %s", s_last_rx_lqi, text);
            return true;
        }

        AppMessage wire_msg;
        memcpy(&wire_msg, ind.asdu, sizeof(wire_msg));
        wire_msg.cmd[CMD_LEN - 1] = '\0';

        mesh_msg_t rx = {0};
        strncpy(rx.source, "coordinator", sizeof(rx.source) - 1);
        strncpy(rx.cmd, wire_msg.cmd, sizeof(rx.cmd) - 1);
        rx.value = wire_msg.value;

        if (!enqueue_incoming_from_zigbee(rx)) {
            ESP_LOGW(TAG, "RX queue full, dropping cmd='%s'", wire_msg.cmd);
        }

        ESP_LOGI(TAG, "APS RX queued cmd='%s' value=%d LQI=%d", wire_msg.cmd, wire_msg.value, s_last_rx_lqi);
        return true;
    }
    return false;
}

static void mesh_tx_task(void *arg)
{
    mesh_msg_t msg;

    while (1) {
        if (xQueueReceive(s_mesh_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            // Drain any leftover semaphore token before transmitting
            if (s_aps_tx_sem) {
                xSemaphoreTake(s_aps_tx_sem, 0);
            }

            bool sent = false;
            if (msg.text[0] != '\0') {
                send_aps_string_to_coordinator(msg.text);
                sent = true;
            } else if (msg.cmd[0] != '\0') {
                send_aps_command_to_coordinator(msg.cmd, msg.value);
                sent = true;
            } else {
                ESP_LOGW(TAG, "Dropping TX with empty cmd/text");
            }

            if (sent && s_aps_tx_sem) {
                // Wait for radio stack to confirm transmission (with 500ms safety timeout)
                if (xSemaphoreTake(s_aps_tx_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
                    ESP_LOGW(TAG, "APS TX confirm timeout (500ms)");
                }
                // Small 15ms guard band to let radio buffers settle between burst packets
                vTaskDelay(pdMS_TO_TICKS(15));
            }
        }
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
                        , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            s_joined = false;
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");

            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Rejoined network from stored state");
                s_joined = true;
                s_steering_retry_count = 0;
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s), falling back to network steering", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            s_joined = true;
            s_steering_retry_count = 0;
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            s_joined = false;
            s_steering_retry_count++;
            if (!esp_zb_bdb_is_factory_new() && s_steering_retry_count >= 10) {
                ESP_LOGW(TAG, "Rejoin failed %d times with stored credentials; performing factory reset to join fresh", s_steering_retry_count);
                esp_zb_factory_reset();
            } else {
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
        }
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);

    ESP_LOGI(TAG, "Received ZCL attr: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
             message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);

    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        bool light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : false;
        ESP_LOGI(TAG, "Coordinator On/Off attr says: %s", light_state ? "On" : "Off");
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        return ESP_OK;
    }
}

static void esp_zb_task(void *pvParameters)
{
#if defined(ZB_ROUTER_ROLE)
    esp_zb_cfg_t zb_nwk_cfg = {0};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zczr_cfg.max_children = 10;
#else
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
#endif

    esp_zb_init(&zb_nwk_cfg);

    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &info);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_aps_data_indication_handler_register(zb_apsde_data_indication_handler);
    esp_zb_aps_data_confirm_handler_register(zb_apsde_data_confirm_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}
