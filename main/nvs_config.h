#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESH_CFG_NAMESPACE      "mesh_cfg"
#define MESH_NAME_MAX_LEN       18
#define DEFAULT_RF_CHANNEL      1
#define DEFAULT_TX_POWER_QDBM   80  // +20.0 dBm (80 * 0.25 dBm)

typedef enum {
    ROLE_UNPROVISIONED = 0,
    ROLE_END_DEVICE    = 1,
    ROLE_COORDINATOR   = 2
} mesh_role_t;

typedef struct {
    mesh_role_t role;
    char        node_name[MESH_NAME_MAX_LEN];
    uint8_t     channel;
    uint8_t     coordinator_mac[6];
    int8_t      tx_power;
    bool        long_range_enabled;
} mesh_config_t;

/**
 * @brief Initialize NVS flash and load the active configuration from the 'mesh_cfg' namespace.
 * Populates factory defaults if keys are not found.
 *
 * @return ESP_OK on success, or an error code from NVS.
 */
esp_err_t nvs_config_init(void);

/**
 * @brief Get a pointer to the in-memory cached configuration.
 */
const mesh_config_t *nvs_config_get(void);

/**
 * @brief Get the configured role (ROLE_UNPROVISIONED, ROLE_END_DEVICE, ROLE_COORDINATOR).
 */
mesh_role_t nvs_config_get_role(void);

/**
 * @brief Set and commit the role to NVS.
 */
esp_err_t nvs_config_set_role(mesh_role_t role);

/**
 * @brief Get the configured human-readable node name.
 */
const char *nvs_config_get_name(void);

/**
 * @brief Set and commit the human-readable node name to NVS (max 16 chars + null terminator).
 */
esp_err_t nvs_config_set_name(const char *name);

/**
 * @brief Get the configured 2.4 GHz RF channel (1..14).
 */
uint8_t nvs_config_get_channel(void);

/**
 * @brief Set and commit the 2.4 GHz RF channel to NVS (1..14).
 */
esp_err_t nvs_config_set_channel(uint8_t channel);

/**
 * @brief Get the learned or configured coordinator MAC address.
 */
esp_err_t nvs_config_get_coord_mac(uint8_t mac[6]);

/**
 * @brief Set and commit the coordinator MAC address to NVS.
 */
esp_err_t nvs_config_set_coord_mac(const uint8_t mac[6]);

/**
 * @brief Get the configured maximum TX power in quarter-dBm units (80 = +20.0 dBm).
 */
int8_t nvs_config_get_tx_power(void);

/**
 * @brief Set and commit maximum TX power in quarter-dBm units to NVS.
 */
esp_err_t nvs_config_set_tx_power(int8_t tx_power);

/**
 * @brief Check if Long Range mode (WIFI_PROTOCOL_LR) is enabled.
 */
bool nvs_config_get_lr_mode(void);

/**
 * @brief Set and commit Long Range mode enable state to NVS.
 */
esp_err_t nvs_config_set_lr_mode(bool enable);

/**
 * @brief Erase the 'mesh_cfg' NVS namespace and restore factory default configuration.
 */
esp_err_t nvs_config_factory_reset(void);

/**
 * @brief Print formatted configuration, MAC address, role, and uptime via mesh_serial.
 */
void nvs_config_show(void);

/**
 * @brief Handle interactive serial CLI commands for workbench provisioning:
 *        SetRole, SetName, SetChannel, SetTxPower, ShowConfig, FactoryReset, etc.
 *
 * @param line Input command string.
 * @return true if command was handled by workbench CLI, false otherwise.
 */
bool nvs_config_handle_cli(const char *line);

/**
 * @brief Schedule an asynchronous system restart after delay_ms milliseconds.
 */
void nvs_config_schedule_restart(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif // NVS_CONFIG_H

