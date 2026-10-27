/*
 * nvs_config.c - Non-Volatile Storage (NVS) Configuration & Workbench CLI
 *
 * Purpose:
 *   Manages persistent non-volatile configuration parameters stored in the 'mesh_cfg'
 *   NVS partition, including device role, logical node name, RF channel, coordinator MAC,
 *   TX output power, and 802.11 Long-Range PHY mode. Also implements the interactive
 *   serial workbench provisioning console.
 *
 * Supported Commands (Workbench Console):
 *   ShowConfig / config
 *     Displays current configuration table including role, name, channel, coordinator MAC,
 *     TX power, and Long-Range PHY mode.
 *
 *   SetRole <Coordinator | EndDevice <NodeName> | Unprovisioned>
 *     Sets node role and identity, commits to NVS, and schedules device reboot after 1 second.
 *
 *   SetName <NewName>
 *     Assigns a persistent logical node name (up to 16 characters).
 *
 *   SetChannel <1-14>
 *     Sets the fixed 2.4GHz Wi-Fi / ESP-NOW channel (1..14).
 *
 *   SetTxPower <qdbm>
 *     Sets Wi-Fi transmission power in quarter-dBm units (8..84, corresponding to +2 dBm to +21 dBm).
 *
 *   SetLR <0|1> / SetLongRange <0|1>
 *     Enables (1) or disables (0) 802.11 Long-Range PHY mode (250 kbps DSSS).
 *
 *   FactoryReset
 *     Erases the 'mesh_cfg' NVS namespace and restarts node in unprovisioned state.
 *
 *   help / ?
 *     Prints available workbench console commands and parameter formats.
 */

#include "nvs_config.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "mesh_serial.h"

static const char *TAG = "NVS_CONFIG";

// NVS Keys
#define KEY_ROLE       "role"
#define KEY_NAME       "name"
#define KEY_CHANNEL    "channel"
#define KEY_COORD_MAC  "coord_mac"
#define KEY_TX_POWER   "tx_power"
#define KEY_LR_MODE    "lr_mode"

// In-memory cached configuration
static mesh_config_t s_config = {
    .role = ROLE_UNPROVISIONED,
    .node_name = "Node",
    .channel = DEFAULT_RF_CHANNEL,
    .coordinator_mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    .tx_power = DEFAULT_TX_POWER_QDBM,
    .long_range_enabled = true,
};

static bool s_initialized = false;

static void set_default_config(void)
{
    s_config.role = ROLE_UNPROVISIONED;
    strncpy(s_config.node_name, "Node", sizeof(s_config.node_name) - 1);
    s_config.node_name[sizeof(s_config.node_name) - 1] = '\0';
    s_config.channel = DEFAULT_RF_CHANNEL;
    memset(s_config.coordinator_mac, 0xFF, sizeof(s_config.coordinator_mac));
    s_config.tx_power = DEFAULT_TX_POWER_QDBM;
    s_config.long_range_enabled = true;
}

esp_err_t nvs_config_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated or updated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    set_default_config();

    nvs_handle_t handle;
    err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace '%s': %s", MESH_CFG_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    // 1. Read Role
    uint8_t role_val = 0;
    if (nvs_get_u8(handle, KEY_ROLE, &role_val) == ESP_OK) {
        if (role_val <= ROLE_COORDINATOR) {
            s_config.role = (mesh_role_t)role_val;
        }
    }

    // 2. Read Name
    size_t name_len = sizeof(s_config.node_name);
    char name_buf[MESH_NAME_MAX_LEN] = {0};
    if (nvs_get_str(handle, KEY_NAME, name_buf, &name_len) == ESP_OK) {
        strncpy(s_config.node_name, name_buf, sizeof(s_config.node_name) - 1);
        s_config.node_name[sizeof(s_config.node_name) - 1] = '\0';
    }

    // 3. Read Channel
    uint8_t ch_val = 0;
    if (nvs_get_u8(handle, KEY_CHANNEL, &ch_val) == ESP_OK) {
        if (ch_val >= 1 && ch_val <= 14) {
            s_config.channel = ch_val;
        }
    }

    // 4. Read Coordinator MAC
    size_t mac_len = sizeof(s_config.coordinator_mac);
    uint8_t mac_buf[6] = {0};
    if (nvs_get_blob(handle, KEY_COORD_MAC, mac_buf, &mac_len) == ESP_OK && mac_len == 6) {
        memcpy(s_config.coordinator_mac, mac_buf, 6);
    }

    // 5. Read TX Power
    int8_t pwr_val = 0;
    if (nvs_get_i8(handle, KEY_TX_POWER, &pwr_val) == ESP_OK) {
        s_config.tx_power = pwr_val;
    }

    // 6. Read Long Range Mode
    uint8_t lr_val = 0;
    if (nvs_get_u8(handle, KEY_LR_MODE, &lr_val) == ESP_OK) {
        s_config.long_range_enabled = (lr_val != 0);
    }

    nvs_close(handle);
    s_initialized = true;

    ESP_LOGI(TAG, "Config loaded: Role=%d, Name='%s', Channel=%u, Power=%d (%.1f dBm), LR=%d",
             s_config.role, s_config.node_name, s_config.channel,
             s_config.tx_power, s_config.tx_power * 0.25f,
             s_config.long_range_enabled);

    return ESP_OK;
}

const mesh_config_t *nvs_config_get(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return &s_config;
}

mesh_role_t nvs_config_get_role(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return s_config.role;
}

esp_err_t nvs_config_set_role(mesh_role_t role)
{
    if (role > ROLE_COORDINATOR) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(handle, KEY_ROLE, (uint8_t)role);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        s_config.role = role;
        ESP_LOGI(TAG, "Role persisted: %d", role);
    }
    return err;
}

const char *nvs_config_get_name(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return s_config.node_name;
}

bool nvs_config_is_name_valid(const char *name)
{
    if (!name) return false;
    size_t len = strlen(name);
    if (len < 2 || len >= MESH_NAME_MAX_LEN) return false;
    if (strcasecmp(name, "Node") == 0 || strcasecmp(name, "Unassigned") == 0 || strcasecmp(name, "default") == 0) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)name[i];
        if (isspace(c) || iscntrl(c) || c == ',' || c == ':' || c == ';') {
            return false;
        }
    }
    return true;
}

esp_err_t nvs_config_set_name(const char *name)
{
    if (!name || strlen(name) == 0 || strlen(name) >= MESH_NAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_str(handle, KEY_NAME, name);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        strncpy(s_config.node_name, name, sizeof(s_config.node_name) - 1);
        s_config.node_name[sizeof(s_config.node_name) - 1] = '\0';
        ESP_LOGI(TAG, "Name persisted: '%s'", s_config.node_name);
    }
    return err;
}

uint8_t nvs_config_get_channel(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return s_config.channel;
}

esp_err_t nvs_config_set_channel(uint8_t channel)
{
    if (channel < 1 || channel > 14) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(handle, KEY_CHANNEL, channel);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        s_config.channel = channel;
        // Also tune live radio if Wi-Fi stack is already running
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        ESP_LOGI(TAG, "Channel persisted & set: %u", channel);
    }
    return err;
}

esp_err_t nvs_config_get_coord_mac(uint8_t mac[6])
{
    if (!mac) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) {
        nvs_config_init();
    }
    memcpy(mac, s_config.coordinator_mac, 6);
    return ESP_OK;
}

esp_err_t nvs_config_set_coord_mac(const uint8_t mac[6])
{
    if (!mac) return ESP_ERR_INVALID_ARG;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, KEY_COORD_MAC, mac, 6);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        memcpy(s_config.coordinator_mac, mac, 6);
        ESP_LOGI(TAG, "Coordinator MAC persisted: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return err;
}

int8_t nvs_config_get_tx_power(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return s_config.tx_power;
}

esp_err_t nvs_config_set_tx_power(int8_t tx_power)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_i8(handle, KEY_TX_POWER, tx_power);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        s_config.tx_power = tx_power;
        esp_wifi_set_max_tx_power(tx_power);
        ESP_LOGI(TAG, "TX power persisted & set: %d (%.1f dBm)", tx_power, tx_power * 0.25f);
    }
    return err;
}

bool nvs_config_get_lr_mode(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }
    return s_config.long_range_enabled;
}

esp_err_t nvs_config_set_lr_mode(bool enable)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(handle, KEY_LR_MODE, enable ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        s_config.long_range_enabled = enable;
        ESP_LOGI(TAG, "Long Range mode persisted: %d", enable ? 1 : 0);
    }
    return err;
}

esp_err_t nvs_config_factory_reset(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MESH_CFG_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        nvs_erase_all(handle);
        nvs_commit(handle);
        nvs_close(handle);
    }
    set_default_config();
    ESP_LOGW(TAG, "Factory reset performed, mesh_cfg erased.");
    return ESP_OK;
}

static void restart_task(void *arg)
{
    uint32_t delay_ms = (uint32_t)(uintptr_t)arg;
    if (delay_ms == 0) delay_ms = 1000;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    esp_restart();
    vTaskDelete(NULL);
}

void nvs_config_schedule_restart(uint32_t delay_ms)
{
    xTaskCreate(restart_task, "restart_task", 2048, (void *)(uintptr_t)delay_ms, 5, NULL);
}

void nvs_config_show(void)
{
    if (!s_initialized) {
        nvs_config_init();
    }

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    const char *role_str = "UNPROVISIONED";
    if (s_config.role == ROLE_END_DEVICE) {
        role_str = "END_DEVICE";
    } else if (s_config.role == ROLE_COORDINATOR) {
        role_str = "COORDINATOR";
    }

    const char *name_str = s_config.node_name;
    if (s_config.role == ROLE_UNPROVISIONED && strcmp(s_config.node_name, "Node") == 0) {
        name_str = "Unassigned";
    }

    float pwr_dbm = s_config.tx_power * 0.25f;
    int64_t uptime_sec = esp_timer_get_time() / 1000000ULL;
    uint32_t hrs = (uint32_t)(uptime_sec / 3600);
    uint32_t mins = (uint32_t)((uptime_sec % 3600) / 60);
    uint32_t secs = (uint32_t)(uptime_sec % 60);

    mesh_serial_writef("[CONFIG] Role: %s (%d)\r\n", role_str, (int)s_config.role);
    mesh_serial_writef("[CONFIG] Name: %s\r\n", name_str);
    mesh_serial_writef("[CONFIG] MAC:  %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    mesh_serial_writef("[CONFIG] RF:   Channel %u, TX Power %d (%+.1f dBm), Long Range: %s\r\n",
                       s_config.channel, s_config.tx_power, pwr_dbm,
                       s_config.long_range_enabled ? "ON" : "OFF");
    mesh_serial_writef("[CONFIG] Uptime: %lu:%02lu:%02lu (%llus)\r\n",
                       (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs,
                       (unsigned long long)uptime_sec);
}

static void print_workbench_help(void)
{
    mesh_serial_write("\r\n=== Workbench Configuration Commands ===\r\n");
    mesh_serial_write("  SetRole Coordinator            -> Configure as Coordinator gateway & reboot\r\n");
    mesh_serial_write("  SetRole EndDevice <NodeName>    -> Configure as End Device with name & reboot\r\n");
    mesh_serial_write("  SetName <NewName>               -> Update node name in NVS\r\n");
    mesh_serial_write("  SetChannel <1-14>               -> Set primary 2.4 GHz RF channel\r\n");
    mesh_serial_write("  SetTxPower <q_dbm>              -> Set TX power (e.g. 80 for +20.0 dBm)\r\n");
    mesh_serial_write("  SetLR <0|1>                     -> Enable/disable Long Range (250K) PHY\r\n");
    mesh_serial_write("  ShowConfig                      -> Display current NVS settings & status\r\n");
    mesh_serial_write("  FactoryReset                    -> Revert to UNPROVISIONED & reboot\r\n\r\n");
}

bool nvs_config_handle_cli(const char *line)
{
    if (!line) return false;

    // Skip leading whitespace
    while (*line && isspace((unsigned char)*line)) {
        line++;
    }
    if (*line == '\0') return false;

    char cmd[32] = {0};
    char arg1[32] = {0};
    char arg2[32] = {0};

    int fields = sscanf(line, "%31s %31s %31s", cmd, arg1, arg2);
    if (fields < 1) return false;

    // --- ShowConfig ---
    if (strcasecmp(cmd, "ShowConfig") == 0 || strcasecmp(cmd, "config") == 0) {
        nvs_config_show();
        return true;
    }

    // --- SetRole ---
    if (strcasecmp(cmd, "SetRole") == 0) {
        if (fields < 2) {
            mesh_serial_write("[ERROR] Usage: SetRole Coordinator | SetRole EndDevice <NodeName> | SetRole Unprovisioned\r\n");
            return true;
        }

        if (strcasecmp(arg1, "Coordinator") == 0 || strcasecmp(arg1, "coord") == 0) {
            const char *coord_name = (fields >= 3 && strlen(arg2) > 0) ? arg2 : "Coordinator";
            mesh_serial_writef("[NVS] Writing name: \"%s\"... ", coord_name);
            nvs_config_set_name(coord_name);
            mesh_serial_write("OK\r\n");

            mesh_serial_write("[NVS] Writing role: COORDINATOR (2)... ");
            if (nvs_config_set_role(ROLE_COORDINATOR) == ESP_OK) {
                mesh_serial_write("OK\r\n");
                mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
                mesh_serial_write("[SYS] Restarting node in 1 second...\r\n");
                nvs_config_schedule_restart(1000);
            } else {
                mesh_serial_write("FAILED\r\n");
            }
            return true;
        }

        if (strcasecmp(arg1, "EndDevice") == 0 || strcasecmp(arg1, "node") == 0) {
            if (fields < 3 || strlen(arg2) == 0) {
                mesh_serial_write("[ERROR] Node name is required when provisioning as EndDevice.\r\n");
                mesh_serial_write("        Usage: SetRole EndDevice <NodeName>\r\n");
                mesh_serial_write("        Example: SetRole EndDevice Kitchen\r\n");
                return true;
            }

            if (!nvs_config_is_name_valid(arg2)) {
                if (strcasecmp(arg2, "Node") == 0 || strcasecmp(arg2, "Unassigned") == 0 || strcasecmp(arg2, "default") == 0) {
                    mesh_serial_writef("[ERROR] '%s' is a reserved default placeholder. Please specify a unique node name.\r\n", arg2);
                } else if (strlen(arg2) < 2) {
                    mesh_serial_write("[ERROR] Node name too short (minimum 2 characters)\r\n");
                } else if (strlen(arg2) >= MESH_NAME_MAX_LEN) {
                    mesh_serial_writef("[ERROR] Node name too long (maximum %d characters)\r\n", MESH_NAME_MAX_LEN - 1);
                } else {
                    mesh_serial_write("[ERROR] Invalid characters in node name (spaces and delimiters not allowed)\r\n");
                }
                return true;
            }

            mesh_serial_writef("[NVS] Writing name: \"%s\"... ", arg2);
            if (nvs_config_set_name(arg2) != ESP_OK) {
                mesh_serial_write("FAILED\r\n");
                return true;
            }
            mesh_serial_write("OK\r\n");

            mesh_serial_write("[NVS] Writing role: END_DEVICE (1)... ");
            if (nvs_config_set_role(ROLE_END_DEVICE) != ESP_OK) {
                mesh_serial_write("FAILED\r\n");
                return true;
            }
            mesh_serial_write("OK\r\n");

            mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
            mesh_serial_write("[SYS] Restarting node in 1 second...\r\n");
            nvs_config_schedule_restart(1000);
            return true;
        }

        if (strcasecmp(arg1, "Unprovisioned") == 0) {
            mesh_serial_write("[NVS] Writing role: UNPROVISIONED (0)... ");
            if (nvs_config_set_role(ROLE_UNPROVISIONED) == ESP_OK) {
                mesh_serial_write("OK\r\n");
                mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
                mesh_serial_write("[SYS] Restarting node in 1 second...\r\n");
                nvs_config_schedule_restart(1000);
            } else {
                mesh_serial_write("FAILED\r\n");
            }
            return true;
        }

        mesh_serial_writef("[ERROR] Unknown role '%s'. Options: Coordinator, EndDevice, Unprovisioned\r\n", arg1);
        return true;
    }

    // --- SetName ---
    if (strcasecmp(cmd, "SetName") == 0) {
        if (fields < 2 || strlen(arg1) == 0) {
            mesh_serial_write("[ERROR] Usage: SetName <NewName>\r\n");
            return true;
        }

        if (!nvs_config_is_name_valid(arg1)) {
            if (strcasecmp(arg1, "Node") == 0 || strcasecmp(arg1, "Unassigned") == 0 || strcasecmp(arg1, "default") == 0) {
                mesh_serial_writef("[ERROR] '%s' is a reserved default placeholder. Please specify a unique node name.\r\n", arg1);
            } else if (strlen(arg1) < 2) {
                mesh_serial_write("[ERROR] Node name too short (minimum 2 characters)\r\n");
            } else if (strlen(arg1) >= MESH_NAME_MAX_LEN) {
                mesh_serial_writef("[ERROR] Node name too long (maximum %d characters)\r\n", MESH_NAME_MAX_LEN - 1);
            } else {
                mesh_serial_write("[ERROR] Invalid characters in node name (spaces and delimiters not allowed)\r\n");
            }
            return true;
        }

        mesh_serial_writef("[NVS] Writing name: \"%s\"... ", arg1);
        if (nvs_config_set_name(arg1) == ESP_OK) {
            mesh_serial_write("OK\r\n");
            mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
        } else {
            mesh_serial_write("FAILED\r\n");
        }
        return true;
    }

    // --- SetChannel ---
    if (strcasecmp(cmd, "SetChannel") == 0) {
        if (fields < 2) {
            mesh_serial_write("[ERROR] Usage: SetChannel <1-14>\r\n");
            return true;
        }

        int ch = atoi(arg1);
        if (ch < 1 || ch > 14) {
            mesh_serial_write("[ERROR] Channel must be between 1 and 14\r\n");
            return true;
        }

        mesh_serial_writef("[NVS] Writing channel: %d... ", ch);
        if (nvs_config_set_channel((uint8_t)ch) == ESP_OK) {
            mesh_serial_write("OK\r\n");
            mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
        } else {
            mesh_serial_write("FAILED\r\n");
        }
        return true;
    }

    // --- SetTxPower ---
    if (strcasecmp(cmd, "SetTxPower") == 0) {
        if (fields < 2) {
            mesh_serial_write("[ERROR] Usage: SetTxPower <quarter_dbm> (e.g. 80 for +20 dBm)\r\n");
            return true;
        }

        int pwr = atoi(arg1);
        if (pwr < 8 || pwr > 84) {
            mesh_serial_write("[ERROR] TX power out of range (8 to 84 quarter-dBm, +2 to +21 dBm)\r\n");
            return true;
        }

        mesh_serial_writef("[NVS] Writing TX power: %d (%+.1f dBm)... ", pwr, pwr * 0.25f);
        if (nvs_config_set_tx_power((int8_t)pwr) == ESP_OK) {
            mesh_serial_write("OK\r\n");
            mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
        } else {
            mesh_serial_write("FAILED\r\n");
        }
        return true;
    }

    // --- SetLR / SetLongRange ---
    if (strcasecmp(cmd, "SetLR") == 0 || strcasecmp(cmd, "SetLongRange") == 0) {
        if (fields < 2) {
            mesh_serial_write("[ERROR] Usage: SetLR <0|1>\r\n");
            return true;
        }

        int lr = atoi(arg1);
        mesh_serial_writef("[NVS] Writing Long Range: %s... ", lr ? "ON" : "OFF");
        if (nvs_config_set_lr_mode(lr != 0) == ESP_OK) {
            mesh_serial_write("OK\r\n");
            mesh_serial_write("[NVS] Configuration committed successfully.\r\n");
        } else {
            mesh_serial_write("FAILED\r\n");
        }
        return true;
    }

    // --- FactoryReset ---
    if (strcasecmp(cmd, "FactoryReset") == 0) {
        mesh_serial_write("[NVS] Erasing 'mesh_cfg' namespace... ");
        if (nvs_config_factory_reset() == ESP_OK) {
            mesh_serial_write("OK\r\n");
            mesh_serial_write("[NVS] Reverted to factory defaults.\r\n");
            mesh_serial_write("[SYS] Restarting node in 1 second...\r\n");
            nvs_config_schedule_restart(1000);
        } else {
            mesh_serial_write("FAILED\r\n");
        }
        return true;
    }

    // --- help / ? ---
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_workbench_help();
        return true;
    }

    return false;
}

