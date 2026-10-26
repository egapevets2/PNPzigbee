#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_DEFAULT_I2C_ADDR 0x3C
#define SSD1306_WIDTH            128
#define SSD1306_HEIGHT           64

/**
 * @brief Initializes the SSD1306 OLED display on the shared I2C bus.
 * Configures the display in 128x64 horizontal addressing mode with internal charge pump.
 * @param i2c_addr 7-bit I2C address (usually 0x3C; pass 0 for default 0x3C).
 */
esp_err_t ssd1306_init(uint8_t i2c_addr);

/**
 * @brief Checks if SSD1306 is currently initialized.
 */
bool ssd1306_is_initialized(void);

/**
 * @brief Clears the entire display buffer and updates the screen to black.
 * Resets the vertical line cursor to the top of the display (y = 0).
 */
esp_err_t ssd1306_clear_display(void);

/**
 * @brief Prints a line of text in FreeMono font (9pt, 12pt, 18pt, or 24pt).
 * Each call clears the line band, draws text, updates the display, and advances
 * the cursor to the next line down. Wraps back to top if the bottom is reached.
 * @param size_pt Point size: 9, 12, 18, or 24.
 * @param text ASCII text string to render.
 */
esp_err_t ssd1306_print_line(int size_pt, const char *text);

/**
 * @brief Renders two lines simultaneously and flushes to display in a single I2C operation.
 * Eliminates screen blanking and multi-packet latency for multi-line status screens.
 * @param size_pt Point size (default 12).
 * @param line1 Text for line 1 (y = 0).
 * @param line2 Text for line 2 (y = yAdvance).
 */
esp_err_t ssd1306_print_2lines(int size_pt, const char *line1, const char *line2);

/**
 * @brief Renders up to 4 lines simultaneously (separated by line index) and flushes to display in a single I2C operation.
 * @param size_pt Point size (default 9 or 12).
 * @param num_lines Number of lines to render (1 to 4).
 * @param lines Array of strings for each line.
 */
esp_err_t ssd1306_print_multilines(int size_pt, int num_lines, const char *lines[]);

/**
 * @brief Renders text at a specific vertical line index without disturbing other lines.
 * @param line_idx 0-based line index (0, 1, 2, ...).
 * @param size_pt Point size (default 12).
 * @param text ASCII text string to render.
 */
esp_err_t ssd1306_print_at(int line_idx, int size_pt, const char *text);

/**
 * @brief Flushes the internal 1024-byte framebuffer to the physical SSD1306.
 */
esp_err_t ssd1306_update(void);

/**
 * @brief Latched-bus detector for SSD1306 OLED display.
 *        Evaluates I2C transmission status and bus line states.
 *        If latched bus detected, sends upstream alert to coordinator and initiates recovery.
 */
bool ssd1306_check_latched_bus(esp_err_t err);

/**
 * @brief Command router checks.
 */
bool ssd1306_is_cmd(const char *cmd);

/**
 * @brief Executes SSD1306 display commands (ClearDisp, print, SetupDisp, SetupSSD1306).
 */
bool ssd1306_execute_cmd(const char *cmd, int16_t value, const char *str_arg, bool text_message);

#ifdef __cplusplus
}
#endif

#endif // SSD1306_H

