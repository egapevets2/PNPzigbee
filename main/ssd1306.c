#include "ssd1306.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_manager.h"
#include "mesh_zigbee.h"

#include "fonts/gfxfont.h"
#include "fonts/FreeMono9pt7b.h"
#include "fonts/FreeMono12pt7b.h"
#include "fonts/FreeMono18pt7b.h"
#include "fonts/FreeMono24pt7b.h"

static const char *TAG = "SSD1306";

static i2c_master_dev_handle_t s_dev_handle = NULL;
static uint8_t s_current_addr = SSD1306_DEFAULT_I2C_ADDR;

// 128x64 Monochrome Framebuffer (1 bit per pixel = 1024 bytes)
static uint8_t s_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Tracks the vertical line cursor (top of current text line, 0 to 63)
static int16_t s_cursor_y = 0;

typedef struct {
    int pt;
    const GFXfont *font;
    int baseline_offset;
} font_spec_t;

static const font_spec_t s_fonts[] = {
    { 9,  &FreeMono9pt7b,  12 },
    { 12, &FreeMono12pt7b, 16 },
    { 18, &FreeMono18pt7b, 24 },
    { 24, &FreeMono24pt7b, 32 }
};

static const font_spec_t *get_font_spec(int size_pt)
{
    for (size_t i = 0; i < sizeof(s_fonts) / sizeof(s_fonts[0]); i++) {
        if (s_fonts[i].pt == size_pt) {
            return &s_fonts[i];
        }
    }
    // If not an exact match, choose closest size
    int best_diff = 999;
    const font_spec_t *best = &s_fonts[1]; // default 12pt
    for (size_t i = 0; i < sizeof(s_fonts) / sizeof(s_fonts[0]); i++) {
        int diff = abs(s_fonts[i].pt - size_pt);
        if (diff < best_diff) {
            best_diff = diff;
            best = &s_fonts[i];
        }
    }
    return best;
}

bool ssd1306_is_initialized(void)
{
    return (s_dev_handle != NULL);
}

void ssd1306_draw_pixel(int16_t x, int16_t y, uint8_t color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int idx = x + (y / 8) * SSD1306_WIDTH;
    if (color) {
        s_buffer[idx] |= (1 << (y & 7));
    } else {
        s_buffer[idx] &= ~(1 << (y & 7));
    }
}

void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    for (int16_t cy = y; cy < y + h; cy++) {
        if (cy < 0 || cy >= SSD1306_HEIGHT) continue;
        for (int16_t cx = x; cx < x + w; cx++) {
            if (cx < 0 || cx >= SSD1306_WIDTH) continue;
            ssd1306_draw_pixel(cx, cy, color);
        }
    }
}

static void ssd1306_draw_char(int16_t x, int16_t y, unsigned char c, const GFXfont *font)
{
    if (c < font->first || c > font->last) return;
    c -= font->first;
    const GFXglyph *glyph = &(font->glyph[c]);
    const uint8_t *bitmap = font->bitmap;

    uint16_t bo = glyph->bitmapOffset;
    uint8_t w = glyph->width;
    uint8_t h = glyph->height;
    int8_t xo = glyph->xOffset;
    int8_t yo = glyph->yOffset;
    uint8_t bits = 0, bit = 0;

    for (uint8_t yy = 0; yy < h; yy++) {
        for (uint8_t xx = 0; xx < w; xx++) {
            if (!(bit++ & 7)) {
                bits = bitmap[bo++];
            }
            if (bits & 0x80) {
                ssd1306_draw_pixel(x + xo + xx, y + yo + yy, 1);
            }
            bits <<= 1;
        }
    }
}

esp_err_t ssd1306_update(void)
{
    if (s_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // 1. Set column address 0 to 127
    uint8_t col_cmd[] = {0x00, 0x21, 0, SSD1306_WIDTH - 1};
    esp_err_t ret = i2c_master_transmit(s_dev_handle, col_cmd, sizeof(col_cmd), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set column address: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. Set page address 0 to 7
    uint8_t page_cmd[] = {0x00, 0x22, 0, (SSD1306_HEIGHT / 8) - 1};
    ret = i2c_master_transmit(s_dev_handle, page_cmd, sizeof(page_cmd), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set page address: %s", esp_err_to_name(ret));
        return ret;
    }

    // 3. Send 1024 bytes of data in 128-byte page packets with 0x40 data prefix
    uint8_t tx_buf[1 + SSD1306_WIDTH];
    tx_buf[0] = 0x40; // Control byte: Co=0, D/C#=1 (Data stream)

    for (int page = 0; page < (SSD1306_HEIGHT / 8); page++) {
        memcpy(&tx_buf[1], &s_buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        ret = i2c_master_transmit(s_dev_handle, tx_buf, sizeof(tx_buf), pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write page %d data: %s", page, esp_err_to_name(ret));
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t ssd1306_init(uint8_t i2c_addr)
{
    if (i2c_manager_init_bus() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus.");
        return ESP_FAIL;
    }

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();
    if (bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus.");
        return ESP_FAIL;
    }

    uint8_t target_addr = (i2c_addr == 0) ? SSD1306_DEFAULT_I2C_ADDR : i2c_addr;

    esp_err_t probe_ret = i2c_master_probe(bus, target_addr, pdMS_TO_TICKS(50));
    if (probe_ret != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 probe at 0x%02X returned %s", target_addr, esp_err_to_name(probe_ret));
        return probe_ret;
    }

    if (s_dev_handle != NULL) {
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = target_addr,
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SSD1306 I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    s_current_addr = target_addr;

    // Standard SSD1306 128x64 Initialization Sequence with Internal Charge Pump
    static const uint8_t init_cmds[] = {
        0x00,             // Control byte: Command stream
        0xAE,             // Display OFF
        0xD5, 0x80,       // Set Display Clock Divide Ratio / Osc Freq (0x80)
        0xA8, 0x3F,       // Set Multiplex Ratio (0x3F for 64 MUX)
        0xD3, 0x00,       // Set Display Offset (0)
        0x40,             // Set Display Start Line (0)
        0x8D, 0x14,       // Charge Pump Setting: Enable (0x14)
        0x20, 0x00,       // Memory Addressing Mode: Horizontal (0x00)
        0xA1,             // Set Segment Re-map: Column 127 mapped to SEG0
        0xC8,             // Set COM Output Scan Direction: Remapped (C8)
        0xDA, 0x12,       // Set COM Pins Hardware Config: Alternative (0x12)
        0x81, 0xCF,       // Set Contrast Control (0xCF)
        0xD9, 0xF1,       // Set Pre-charge Period (0xF1)
        0xDB, 0x40,       // Set VCOMH Deselect Level (0x40)
        0xA4,             // Entire Display ON: Resume to RAM content
        0xA6,             // Set Normal Display (non-inverted)
        0x2E,             // Deactivate scroll
        0xAF              // Display ON
    };

    ret = i2c_master_transmit(s_dev_handle, init_cmds, sizeof(init_cmds), pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send initialization commands to SSD1306: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SSD1306 initialized at 0x%02X (128x64)", s_current_addr);

    // Clear buffer and update display
    memset(s_buffer, 0, sizeof(s_buffer));
    s_cursor_y = 0;
    return ssd1306_update();
}

esp_err_t ssd1306_clear_display(void)
{
    if (s_dev_handle == NULL) {
        esp_err_t err = ssd1306_init(s_current_addr);
        if (err != ESP_OK) return err;
    }

    memset(s_buffer, 0, sizeof(s_buffer));
    s_cursor_y = 0; // Reset line cursor back to top
    return ssd1306_update();
}

esp_err_t ssd1306_print_line(int size_pt, const char *text)
{
    if (s_dev_handle == NULL) {
        esp_err_t err = ssd1306_init(s_current_addr);
        if (err != ESP_OK) return err;
    }
    if (!text) return ESP_ERR_INVALID_ARG;

    const font_spec_t *fspec = get_font_spec(size_pt);
    const GFXfont *font = fspec->font;
    int yAdvance = font->yAdvance;
    int baseline = fspec->baseline_offset;

    // Wrap back to top if the next line exceeds the 64-pixel display height
    if (s_cursor_y + yAdvance > SSD1306_HEIGHT) {
        s_cursor_y = 0;
    }

    // Clear the line rectangular strip across full width
    ssd1306_fill_rect(0, s_cursor_y, SSD1306_WIDTH, yAdvance, 0);

    // Render characters
    int16_t x = 0;
    size_t len = strlen(text);
    for (size_t i = 0; i < len; i++) {
        char c = text[i];
        if (c < font->first || c > font->last) {
            if (c == '\r' || c == '\n') break;
            c = ' ';
        }
        const GFXglyph *glyph = &(font->glyph[c - font->first]);
        if (x + glyph->xOffset + glyph->width > SSD1306_WIDTH && i > 0) {
            break; // Stop at right edge
        }
        ssd1306_draw_char(x, s_cursor_y + baseline, (unsigned char)c, font);
        x += glyph->xAdvance;
    }

    // Advance cursor to next line down
    s_cursor_y += yAdvance;

    return ssd1306_update();
}

bool ssd1306_is_cmd(const char *cmd)
{
    if (!cmd) return false;
    return (strcasecmp(cmd, "ClearDisp") == 0 ||
            strcasecmp(cmd, "print") == 0 ||
            strcasecmp(cmd, "SetupDisp") == 0 ||
            strcasecmp(cmd, "SetupSSD1306") == 0);
}

bool ssd1306_execute_cmd(const char *cmd, int16_t value, const char *str_arg, bool text_message)
{
    if (strcasecmp(cmd, "ClearDisp") == 0) {
        if (ssd1306_clear_display() == ESP_OK) {
            mesh_zigbee_send_text("CLEARDISP OK");
        } else {
            mesh_zigbee_send_text("CLEARDISP FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "print") == 0) {
        int size_pt = (value > 0) ? (int)value : 12;
        const char *text = (str_arg && str_arg[0] != '\0') ? str_arg : "";
        if (ssd1306_print_line(size_pt, text) == ESP_OK) {
            mesh_zigbee_send_text("PRINT OK");
        } else {
            mesh_zigbee_send_text("PRINT FAIL");
        }
        return true;
    }
    else if (strcasecmp(cmd, "SetupDisp") == 0 || strcasecmp(cmd, "SetupSSD1306") == 0) {
        uint8_t addr = (value > 0) ? (uint8_t)value : SSD1306_DEFAULT_I2C_ADDR;
        if (ssd1306_init(addr) == ESP_OK) {
            mesh_zigbee_send_text("SETUP DISP OK");
        } else {
            mesh_zigbee_send_text("SETUP DISP FAIL");
        }
        return true;
    }
    return false;
}

