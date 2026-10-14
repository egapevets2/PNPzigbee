#include "i2c_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"

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

