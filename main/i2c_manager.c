/*
 * i2c_manager.c - Centralized Shared I2C Master Bus Resource Manager
 *
 * Purpose:
 *   Manages the lifecycle and shared access to the ESP32-C6 I2C master bus (ESP-IDF
 *   i2c_master driver) on SDA=GPIO 8 and SCL=GPIO 9 at 100 kHz. Peripherals dynamically
 *   attach and release device handles onto this shared bus (SSD1306, APDS9930, VL53L1X,
 *   Compass, PCF8574, I2Craw) without pin conflicts.
 *
 * Supported Commands:
 *   None (Internal Subsystem/Driver)
 */

#include "i2c_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"
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

esp_err_t i2c_manager_deinit_bus(void)
{
    if (s_i2c_bus_handle == NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deleting shared I2C bus and releasing GPIO %d & %d to standard GPIO mode...",
             I2C_DEFAULT_SDA_IO, I2C_DEFAULT_SCL_IO);

    esp_err_t ret = i2c_del_master_bus(s_i2c_bus_handle);
    s_i2c_bus_handle = NULL;

    // Reset pins back to default unconfigured GPIO state
    gpio_reset_pin(I2C_DEFAULT_SDA_IO);
    gpio_reset_pin(I2C_DEFAULT_SCL_IO);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SDA/SCL pins released back to standard GPIO mode.");
    return ESP_OK;
}

bool i2c_manager_is_bus_latched(int *sda_level, int *scl_level)
{
    int sda = gpio_get_level(I2C_DEFAULT_SDA_IO);
    int scl = gpio_get_level(I2C_DEFAULT_SCL_IO);
    if (sda_level) *sda_level = sda;
    if (scl_level) *scl_level = scl;
    return (sda == 0 || scl == 0);
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

    // If still latched or bus_reset wasn't enough, perform manual 9-clock recovery
    ESP_LOGW(TAG, "Performing manual 9-clock SCL pulse recovery...");
    if (s_i2c_bus_handle != NULL) {
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }

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

    // Generate STOP condition
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

    // Re-initialize master bus
    i2c_manager_init_bus();

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

