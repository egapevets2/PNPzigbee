/*
 * i2c_manager.c - Centralized Shared I2C Master Bus Resource Manager
 *
 * Purpose:
 *   Manages on-demand initialization and shared access to the ESP32-C6 I2C master bus (ESP-IDF
 *   i2c_master driver) on SDA=GPIO 22 and SCL=GPIO 23 at 100 kHz.
 *   - Leaves GPIO 22 and GPIO 23 untouched at boot (zero-I2C pin leakage) until an I2C peripheral
 *     is explicitly initialized by command (SSD1306, APDS9930, VL53L1X, Compass, PCF8574, TCA9548A, I2Craw).
 *   - Provides physical bus health diagnostics (i2c_manager_check_latched_bus) that detect stuck-LOW
 *     SDA/SCL lines, transmit rate-limited upstream alerts ("I2C BUS LATCHED: <DEVICE> (SDA=<0|1> SCL=<0|1>)")
 *     to the Coordinator, and execute 9-clock SCL pulse + STOP bit hardware recovery (i2c_manager_recover_bus)
 *     in-place without destroying existing device handles.
 *
 * Supported Commands:
 *   None (Internal Subsystem/Driver)
 */

#include "i2c_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "esp_rom_gpio.h"
#include "hal/i2c_periph.h"
#include "soc/io_mux_reg.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "UpstreamQ.h"
#include "mesh_zigbee.h"

static const char *TAG = "I2C_MGR";

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;

bool i2c_manager_is_initialized(void)
{
    return (s_i2c_bus_handle != NULL);
}

esp_err_t i2c_manager_init_bus(void)
{
    if (s_i2c_bus_handle != NULL) {
        return ESP_OK; // Already active
    }

    ESP_LOGI(TAG, "Initializing shared I2C bus on-demand: SDA=GPIO %d, SCL=GPIO %d @ %d Hz",
             I2C_DEFAULT_SDA_IO, I2C_DEFAULT_SCL_IO, I2C_DEFAULT_FREQ);

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_DEFAULT_NUM,
        .scl_io_num = I2C_DEFAULT_SCL_IO,
        .sda_io_num = I2C_DEFAULT_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        s_i2c_bus_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "Shared I2C master bus initialized. SDA and SCL configured into I2C mode.");
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_manager_get_bus(void)
{
    return s_i2c_bus_handle;
}

i2c_master_bus_handle_t get_shared_i2c_bus(void)
{
    if (s_i2c_bus_handle == NULL) {
        if (i2c_manager_init_bus() != ESP_OK) {
            return NULL;
        }
    }
    return s_i2c_bus_handle;
}

bool i2c_manager_is_bus_latched(int *sda_level, int *scl_level)
{
    int sda = gpio_get_level(I2C_DEFAULT_SDA_IO);
    int scl = gpio_get_level(I2C_DEFAULT_SCL_IO);
    if (sda_level) *sda_level = sda;
    if (scl_level) *scl_level = scl;
    return (sda == 0 || scl == 0);
}

/*
 * Re-binds SDA (GPIO 22) and SCL (GPIO 23) back to the I2C_NUM_0 peripheral
 * signals in the ESP32-C6 GPIO matrix after manual open-drain bit-banging,
 * without deleting s_i2c_bus_handle or invalidating attached device handles.
 */
static void s_restore_i2c_gpio_matrix(void)
{
    const int port_id = I2C_DEFAULT_NUM;

    gpio_set_level(I2C_DEFAULT_SDA_IO, 1);
    gpio_input_enable(I2C_DEFAULT_SDA_IO);
    gpio_od_enable(I2C_DEFAULT_SDA_IO);
    gpio_pullup_en(I2C_DEFAULT_SDA_IO);
    gpio_pulldown_dis(I2C_DEFAULT_SDA_IO);
    gpio_func_sel(I2C_DEFAULT_SDA_IO, PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(I2C_DEFAULT_SDA_IO, i2c_periph_signal[port_id].sda_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(I2C_DEFAULT_SDA_IO, i2c_periph_signal[port_id].sda_in_sig, 0);

    gpio_set_level(I2C_DEFAULT_SCL_IO, 1);
    gpio_input_enable(I2C_DEFAULT_SCL_IO);
    gpio_od_enable(I2C_DEFAULT_SCL_IO);
    gpio_pullup_en(I2C_DEFAULT_SCL_IO);
    gpio_pulldown_dis(I2C_DEFAULT_SCL_IO);
    gpio_func_sel(I2C_DEFAULT_SCL_IO, PIN_FUNC_GPIO);
    esp_rom_gpio_connect_out_signal(I2C_DEFAULT_SCL_IO, i2c_periph_signal[port_id].scl_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(I2C_DEFAULT_SCL_IO, i2c_periph_signal[port_id].scl_in_sig, 0);
}

esp_err_t i2c_manager_recover_bus(void)
{
    ESP_LOGW(TAG, "Attempting I2C bus recovery and reset...");

    if (s_i2c_bus_handle != NULL) {
        esp_err_t r = i2c_master_bus_reset(s_i2c_bus_handle);
        if (r == ESP_OK) {
            int sda = gpio_get_level(I2C_DEFAULT_SDA_IO);
            int scl = gpio_get_level(I2C_DEFAULT_SCL_IO);
            if (sda == 1 && scl == 1) {
                ESP_LOGI(TAG, "i2c_master_bus_reset successfully released bus (SDA=1, SCL=1)");
                return ESP_OK;
            }
        }
    }

    // If still latched or hardware bus_reset timed out, perform manual 9-clock recovery
    // in-place WITHOUT calling i2c_del_master_bus() (which would fail while peripheral
    // device handles are attached and orphan the bus).
    ESP_LOGW(TAG, "Performing manual 9-clock SCL pulse recovery (preserving bus & device handles)...");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_DEFAULT_SDA_IO) | (1ULL << I2C_DEFAULT_SCL_IO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(I2C_DEFAULT_SDA_IO, 1);
    gpio_set_level(I2C_DEFAULT_SCL_IO, 1);
    esp_rom_delay_us(10);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_DEFAULT_SCL_IO, 0);
        esp_rom_delay_us(10);
        gpio_set_level(I2C_DEFAULT_SCL_IO, 1);
        esp_rom_delay_us(10);
        if (gpio_get_level(I2C_DEFAULT_SDA_IO) == 1) {
            break;
        }
    }

    // Generate STOP condition (SDA LOW -> HIGH while SCL is HIGH)
    gpio_set_level(I2C_DEFAULT_SCL_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_DEFAULT_SDA_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_DEFAULT_SCL_IO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(I2C_DEFAULT_SDA_IO, 1);
    esp_rom_delay_us(10);

    int sda = gpio_get_level(I2C_DEFAULT_SDA_IO);
    int scl = gpio_get_level(I2C_DEFAULT_SCL_IO);

    // Reconnect SDA/SCL pins back to I2C_NUM_0 in the GPIO matrix and reset hardware FSM
    if (s_i2c_bus_handle != NULL) {
        s_restore_i2c_gpio_matrix();
        (void)i2c_master_bus_reset(s_i2c_bus_handle);
    } else {
        i2c_manager_init_bus();
    }

    if (sda == 1 && scl == 1) {
        ESP_LOGI(TAG, "Manual 9-clock I2C recovery succeeded (SDA=1, SCL=1)");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "I2C bus remains latched after recovery (SDA=%d, SCL=%d)", sda, scl);
        return ESP_FAIL;
    }
}

static bool s_bus_latched_reported = false;
static int64_t s_last_report_us = 0;

bool i2c_manager_check_latched_bus(const char *device_name, esp_err_t err)
{
    if (err == ESP_OK) {
        if (s_bus_latched_reported) {
            int sda = gpio_get_level(I2C_DEFAULT_SDA_IO);
            int scl = gpio_get_level(I2C_DEFAULT_SCL_IO);
            if (sda == 1 && scl == 1) {
                s_bus_latched_reported = false;
                ESP_LOGI(TAG, "I2C bus healthy and cleared by %s", device_name ? device_name : "I2C");
            }
        }
        return false;
    }

    int sda = 1, scl = 1;
    bool latched = i2c_manager_is_bus_latched(&sda, &scl);

    // Latched condition: line(s) held LOW, or transaction timeout / invalid state
    if (latched || err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "LATCHED BUS DETECTED by device '%s'! (err: %s, SDA=%d, SCL=%d)",
                 device_name ? device_name : "UNKNOWN", esp_err_to_name(err), sda, scl);

        int64_t now_us = esp_timer_get_time();
        if (!s_bus_latched_reported || (now_us - s_last_report_us > 3000000LL)) {
            s_bus_latched_reported = true;
            s_last_report_us = now_us;

            char msg[64];
            snprintf(msg, sizeof(msg), "I2C BUS LATCHED: %s (SDA=%d SCL=%d)",
                     device_name ? device_name : "UNKNOWN", sda, scl);

            if (!SendTheMessage(msg)) {
                mesh_zigbee_send_text(msg);
            }
        }

        i2c_manager_recover_bus();
        return true;
    }

    return false;
}

