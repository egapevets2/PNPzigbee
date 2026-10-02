#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "NeoPixel.h"
#include "mesh_zigbee.h"

#define MAX_PIXELS 20

static const char *TAG = "NEOPIXEL";
static uint8_t pixel_ram[MAX_PIXELS][3] = {0};
static int current_gpio = -1;
static led_strip_handle_t led_strip = NULL;

void NeoPixel_SetGPIO(int gpio_num) 
{
    if (current_gpio == gpio_num) return;

    ESP_LOGI(TAG, "Configuring NeoPixel on GPIO %d", gpio_num);
    current_gpio = gpio_num;

    if (led_strip) {
        led_strip_del(led_strip);
        led_strip = NULL;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = current_gpio,
        .max_leds = MAX_PIXELS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false, }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, 
        .flags = { .with_dma = false, }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

void NeoPixel_SetRed(int index, int val) 
{
    if (index >= 0 && index < MAX_PIXELS) {
        pixel_ram[index][0] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_SetGreen(int index, int val) 
{
    if (index >= 0 && index < MAX_PIXELS) {
        pixel_ram[index][1] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_SetBlue(int index, int val) 
{
    if (index >= 0 && index < MAX_PIXELS) {
        pixel_ram[index][2] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_Update(void) 
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel Update called before setting GPIO (NeoGPIO)");
        return;
    }

    for (int i = 0; i < MAX_PIXELS; i++) {
        led_strip_set_pixel(led_strip, i, pixel_ram[i][0], pixel_ram[i][1], pixel_ram[i][2]);
    }
    
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "NeoPixels physical strip updated from RAM");
}

bool neopixel_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "NeoGPIO") == 0 || strcmp(cmd, "NeoRED") == 0 ||
            strcmp(cmd, "NeoGREEN") == 0 || strcmp(cmd, "NeoBLUE") == 0 ||
            strcmp(cmd, "NeoUpdate") == 0);
}

bool neopixel_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcmp(cmd, "NeoGPIO") == 0) {
        NeoPixel_SetGPIO(value);
        if (!text_message) mesh_zigbee_send_text("GOT NEOGPIO OK");
        return true;
    } else if (strcmp(cmd, "NeoRED") == 0) {
        NeoPixel_SetRed(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEORED OK");
        return true;
    } else if (strcmp(cmd, "NeoGREEN") == 0) {
        NeoPixel_SetGreen(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEOGREEN OK");
        return true;
    } else if (strcmp(cmd, "NeoBLUE") == 0) {
        NeoPixel_SetBlue(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEOBLUE OK");
        return true;
    } else if (strcmp(cmd, "NeoUpdate") == 0) {
        NeoPixel_Update();
        if (!text_message) mesh_zigbee_send_text("GOT NEOUPDATE OK");
        return true;
    }
    return false;
}