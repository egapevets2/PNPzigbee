/*
 * NeoPixel.c - WS2812 / NeoPixel Addressable LED Strip Driver
 *
 * Purpose:
 *   Drives WS2812 / WS2812B addressable RGB LED strips using the ESP32-C6 GPSPI2 peripheral
 *   with hardware General Purpose DMA (GDMA) at 2.5 MHz resolution. Streaming directly from DMA
 *   buffers guarantees zero CPU interrupt starvation and exact bit timing, completely eliminating
 *   glitching and color latch errors even when the CPU runs at 80 MHz alongside Wi-Fi/ESP-NOW
 *   interrupts. Supports individual pixel color buffering in RAM (up to MAX_PIXELS = 100) with
 *   deferred batch update, bulk clearing, and atomic single-command color presets.
 *
 * Supported Commands:
 *   NeoGPIO <gpio> [num_pixels]
 *     - gpio: GPIO pin connected to NeoPixel data input line (routed via GPIO Matrix to SPI MOSI).
 *     - num_pixels: Number of pixels in the strip (1..100, default 16).
 *     Initializes SPI+DMA driver and clears all pixels to black.
 *     Replies "GOT NEOGPIO OK".
 *
 *   NeoRED <index> <val>
 *     - index: Pixel zero-based index (0..num_pixels-1).
 *     - val: Red brightness (0..255).
 *     Buffers red channel in RAM; requires 'NeoUpdate' to display.
 *     Replies "GOT NEORED OK".
 *
 *   NeoGREEN <index> <val>
 *     - index: Pixel zero-based index (0..num_pixels-1).
 *     - val: Green brightness (0..255).
 *     Buffers green channel in RAM; requires 'NeoUpdate' to display.
 *     Replies "GOT NEOGREEN OK".
 *
 *   NeoBLUE <index> <val>
 *     - index: Pixel zero-based index (0..num_pixels-1).
 *     - val: Blue brightness (0..255).
 *     Buffers blue channel in RAM; requires 'NeoUpdate' to display.
 *     Replies "GOT NEOBLUE OK".
 *
 *   NeoUpdate
 *     Flushes the entire RAM buffer to the physical LED strip via SPI DMA.
 *     Replies "GOT NEOUPDATE OK".
 *
 *   NeoOff <index>
 *     - index: Pixel zero-based index to turn OFF (0,0,0) and immediately update strip.
 *     Replies "GOT NEOOFF OK".
 *
 *   NeoClear
 *     Clears all pixels in RAM and physically turns off the entire strip.
 *     Replies "GOT NEOCLEAR OK".
 *
 *   NeoRed <index> <val> / NeoSetRed <index> <val>
 *     - Sets pixel <index> to RED=<val>, GREEN=0, BLUE=0 and immediately refreshes strip.
 *     Replies "GOT NEORED OK".
 *
 *   NeoGreen <index> <val> / NeoSetGreen <index> <val>
 *     - Sets pixel <index> to RED=0, GREEN=<val>, BLUE=0 and immediately refreshes strip.
 *     Replies "GOT NEOGREEN OK".
 *
 *   NeoBlue <index> <val> / NeoSetBlue <index> <val>
 *     - Sets pixel <index> to RED=0, GREEN=0, BLUE=<val> and immediately refreshes strip.
 *     Replies "GOT NEOBLUE OK".
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_spi.h"
#include "driver/gpio.h"
#include "NeoPixel.h"
#include "mesh_zigbee.h"

#define MAX_PIXELS 100
#define DEFAULT_PIXELS 16

static const char *TAG = "NEOPIXEL";
static uint8_t pixel_ram[MAX_PIXELS][3] = {0};
static int current_gpio = -1;
static int current_num_pixels = DEFAULT_PIXELS;
static led_strip_handle_t led_strip = NULL;

void NeoPixel_SetGPIO(int gpio_num, int num_pixels) 
{
    if (num_pixels <= 0) {
        num_pixels = (current_num_pixels > 0) ? current_num_pixels : DEFAULT_PIXELS;
    }
    if (num_pixels > MAX_PIXELS) {
        ESP_LOGW(TAG, "Requested %d pixels exceeds MAX_PIXELS (%d), clamping", num_pixels, MAX_PIXELS);
        num_pixels = MAX_PIXELS;
    }

    if (current_gpio == gpio_num && current_num_pixels == num_pixels && led_strip != NULL) return;

    ESP_LOGI(TAG, "Configuring NeoPixel on GPIO %d with %d pixels", gpio_num, num_pixels);
    current_gpio = gpio_num;
    current_num_pixels = num_pixels;

    if (led_strip) {
        led_strip_del(led_strip);
        led_strip = NULL;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = current_gpio,
        .max_leds = current_num_pixels,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false, }
    };

    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI2_HOST,
        .flags = {
            .with_dma = true,
        }
    };

    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    
    // Clear RAM buffer for active pixels
    for (int i = 0; i < current_num_pixels; i++) {
        pixel_ram[i][0] = 0;
        pixel_ram[i][1] = 0;
        pixel_ram[i][2] = 0;
    }
    led_strip_clear(led_strip);
}

int NeoPixel_GetNumPixels(void)
{
    return current_num_pixels;
}

void NeoPixel_SetRed(int index, int val) 
{
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][0] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_SetGreen(int index, int val) 
{
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][1] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_SetBlue(int index, int val) 
{
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][2] = (val > 255) ? 255 : (val < 0 ? 0 : val);
    }
}

void NeoPixel_Update(void) 
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel Update called before setting GPIO (NeoGPIO)");
        return;
    }

    for (int i = 0; i < current_num_pixels; i++) {
        led_strip_set_pixel(led_strip, i, pixel_ram[i][0], pixel_ram[i][1], pixel_ram[i][2]);
    }
    
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, "NeoPixels physical strip updated from RAM (%d pixels)", current_num_pixels);
}

void NeoPixel_TurnOff(int index) 
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel TurnOff called before setting GPIO (NeoGPIO)");
        return;
    }
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][0] = 0;
        pixel_ram[index][1] = 0;
        pixel_ram[index][2] = 0;
        NeoPixel_Update();
        ESP_LOGI(TAG, "NeoPixel %d turned OFF", index);
    }
}

void NeoPixel_Clear(void)
{
    for (int i = 0; i < current_num_pixels; i++) {
        pixel_ram[i][0] = 0;
        pixel_ram[i][1] = 0;
        pixel_ram[i][2] = 0;
    }
    if (led_strip) {
        led_strip_clear(led_strip);
        ESP_LOGI(TAG, "All NeoPixels cleared to black (%d pixels)", current_num_pixels);
    }
}

void NeoPixel_SetOnlyRed(int index, int val)
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel SetOnlyRed called before setting GPIO (NeoGPIO)");
        return;
    }
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][0] = (val > 255) ? 255 : (val < 0 ? 0 : val);
        pixel_ram[index][1] = 0;
        pixel_ram[index][2] = 0;
        NeoPixel_Update();
        ESP_LOGI(TAG, "NeoPixel %d set to RED=%d, GREEN=0, BLUE=0", index, pixel_ram[index][0]);
    }
}

void NeoPixel_SetOnlyGreen(int index, int val)
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel SetOnlyGreen called before setting GPIO (NeoGPIO)");
        return;
    }
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][0] = 0;
        pixel_ram[index][1] = (val > 255) ? 255 : (val < 0 ? 0 : val);
        pixel_ram[index][2] = 0;
        NeoPixel_Update();
        ESP_LOGI(TAG, "NeoPixel %d set to RED=0, GREEN=%d, BLUE=0", index, pixel_ram[index][1]);
    }
}

void NeoPixel_SetOnlyBlue(int index, int val)
{
    if (!led_strip) {
        ESP_LOGW(TAG, "NeoPixel SetOnlyBlue called before setting GPIO (NeoGPIO)");
        return;
    }
    if (index >= 0 && index < current_num_pixels) {
        pixel_ram[index][0] = 0;
        pixel_ram[index][1] = 0;
        pixel_ram[index][2] = (val > 255) ? 255 : (val < 0 ? 0 : val);
        NeoPixel_Update();
        ESP_LOGI(TAG, "NeoPixel %d set to RED=0, GREEN=0, BLUE=%d", index, pixel_ram[index][2]);
    }
}

bool neopixel_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcmp(cmd, "NeoGPIO") == 0 || strcmp(cmd, "NeoRED") == 0 ||
            strcmp(cmd, "NeoGREEN") == 0 || strcmp(cmd, "NeoBLUE") == 0 ||
            strcmp(cmd, "NeoUpdate") == 0 || strcmp(cmd, "NeoOff") == 0 ||
            strcmp(cmd, "NeoClear") == 0 ||
            strcmp(cmd, "NeoRed") == 0 || strcmp(cmd, "NeoSetRed") == 0 ||
            strcmp(cmd, "NeoGreen") == 0 || strcmp(cmd, "NeoSetGreen") == 0 ||
            strcmp(cmd, "NeoBlue") == 0 || strcmp(cmd, "NeoSetBlue") == 0);
}

bool neopixel_execute_cmd(const char *cmd, int16_t value, int16_t value2, bool text_message)
{
    if (strcmp(cmd, "NeoGPIO") == 0) {
        int pixels = (value2 > 0) ? value2 : DEFAULT_PIXELS;
        NeoPixel_SetGPIO(value, pixels);
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
    } else if (strcmp(cmd, "NeoOff") == 0) {
        NeoPixel_TurnOff(value);
        if (!text_message) mesh_zigbee_send_text("GOT NEOOFF OK");
        return true;
    } else if (strcmp(cmd, "NeoClear") == 0) {
        NeoPixel_Clear();
        if (!text_message) mesh_zigbee_send_text("GOT NEOCLEAR OK");
        return true;
    } else if (strcmp(cmd, "NeoRed") == 0 || strcmp(cmd, "NeoSetRed") == 0) {
        NeoPixel_SetOnlyRed(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEORED OK");
        return true;
    } else if (strcmp(cmd, "NeoGreen") == 0 || strcmp(cmd, "NeoSetGreen") == 0) {
        NeoPixel_SetOnlyGreen(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEOGREEN OK");
        return true;
    } else if (strcmp(cmd, "NeoBlue") == 0 || strcmp(cmd, "NeoSetBlue") == 0) {
        NeoPixel_SetOnlyBlue(value, value2);
        if (!text_message) mesh_zigbee_send_text("GOT NEOBLUE OK");
        return true;
    }
    return false;
}