// File: esp_zb_light.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "light_driver.h"
#include "pwm_driver.h"
#include "mesh_serial.h"
#include "mesh_zigbee.h"
#include "serial_bridge.h"
#include "apds9930.h"
#include "motor_driver.h"
#include "UpstreamQ.h"
#include "GPIO_handler.h"
#include "adc.h"
#include "Stepper.h"
#include "NeoPixel.h"
#include "BLDC_driver.h"
#include "ledc_manager.h"
#include "VL53L1X.h"
#include "I2Craw.h"

static const char *TAG = "LIGHT_APP";

#define BUTTON_GPIO GPIO_NUM_9
#define BUTTON_POLL_MS 20

static void strip_line_endings(char *s)
{
    if (!s) return;
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n')) {
        s[len - 1] = '\0';
        len--;
    }
}

static bool parse_text_for_local_action(const char *line, char *cmd, size_t cmd_len, int16_t *value1, int16_t *value2)
{
    if (!line || !cmd || cmd_len == 0 || !value1 || !value2) return false;

    char first[MESH_TEXT_LEN] = {0};
    char second[MESH_TEXT_LEN] = {0};
    int p1 = 0;
    int p2 = 0;

    int fields = sscanf(line, "%17s %17s %d %d", first, second, &p1, &p2);
    if (fields < 1) return false;

    *value1 = 0;
    *value2 = 0;

    // NO TARGET: First word is the command
    if (strcmp(first, "blink") == 0 || strcmp(first, "on") == 0 || strcmp(first, "off") == 0 ||
        strcmp(first, "SetupSerialBridge") == 0 ||
        
        // Delegated Command Validation
        pwm_driver_is_cmd(first)   ||
        motor_driver_is_cmd(first) ||
        gpio_handler_is_cmd(first) ||
        neopixel_is_cmd(first)     ||
        adc_is_cmd(first)          ||
        BLDC_driver_is_cmd(first)  ||
        stepper_is_cmd(first)      ||
        proximity_is_cmd(first)    ||
        vl53l1x_is_cmd(first)      ||
        I2Craw_is_cmd(first))
    {
        strncpy(cmd, first, cmd_len - 1);
        if (fields >= 2) *value1 = (int16_t)atoi(second);
        if (fields >= 3) *value2 = (int16_t)p1;
    }
    // HAS TARGET: First word is target, second is command
    else if (fields >= 2)
    {
        strncpy(cmd, second, cmd_len - 1);
        if (fields >= 3) *value1 = (int16_t)p1;
        if (fields >= 4) *value2 = (int16_t)p2;
    }
    else
    {
        return false;
    }

    cmd[cmd_len - 1] = '\0';
    return true;
}

static void serial_console_task(void *arg)
{
    char line[MESH_LINE_LEN] = {0};
    size_t pos = 0;

    ESP_LOGI(TAG, "Serial upstream interface ready: type text to send to coordinator");
    mesh_serial_write("\r\nSerial ready. Type text to send to coordinator.\r\n");

    while (1)
    {
        int ch = mesh_serial_read_byte();
        if (ch < 0) continue;

        if (ch == '\r' || ch == '\n') {
            if (pos == 0) continue;

            line[pos] = '\0';
            strip_line_endings(line);
            mesh_serial_writef("\r\n> %s\r\n", line);

            if (!mesh_zigbee_send_text(line)) {
                mesh_serial_write("error: TX queue full\r\n");
            } else {
                mesh_serial_write("queued\r\n");
            }

            pos = 0;
            memset(line, 0, sizeof(line));
        }
        else if (ch == 8 || ch == 127) {
            if (pos > 0) {
                pos--;
                line[pos] = '\0';
                mesh_serial_write("\b \b");
            }
        }
        else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)ch;
            char echo[2] = {(char)ch, 0};
            mesh_serial_write(echo);
        }
    }
}

static void button_task(void *arg)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            if (mesh_zigbee_is_joined()) {
                mesh_zigbee_send_cmd("button", 1);
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void app_msg_task(void *arg)
{
    mesh_msg_t msg;

    while (1)
    {
        if (mesh_zigbee_receive(&msg, portMAX_DELAY))
        {
            bool text_message = (msg.text[0] != '\0');

            if (text_message)
            {
                mesh_serial_writef("< %s: %s\r\n", msg.source, msg.text);
                serial_bridge_write_line(msg.text);

                if (strcmp(msg.text, "PING") == 0) {
                    char pong[MESH_LINE_LEN] = {0};
                    snprintf(pong, sizeof(pong), "PONG LQI=%d", mesh_zigbee_last_lqi());
                    mesh_zigbee_send_text(pong);
                    continue;
                }

                char text_cmd[MESH_TEXT_LEN] = {0};
                int16_t text_value1 = 0;
                int16_t text_value2 = 0;

                if (parse_text_for_local_action(msg.text, text_cmd, sizeof(text_cmd), &text_value1, &text_value2)) {
                    strncpy(msg.cmd, text_cmd, sizeof(msg.cmd) - 1);
                    msg.value = text_value1;
                    msg.value2 = text_value2;
                } else {
                    continue;
                }

                // If the command is ping or setDAC, the downstream Arduino will generate the response!
                // Do NOT send an ACK here so it doesn't conflict or duplicate the Arduino's reply.
                if (strcasecmp(msg.cmd, "ping") != 0 && strcasecmp(msg.cmd, "pingx") != 0 && strcasecmp(msg.cmd, "setDAC") != 0) {
                    if (strcmp(msg.text, "ACK") != 0 && msg.source[0] != '\0') {
                        mesh_zigbee_send_text("ACK");
                    }
                }
            }

            if (strcmp(msg.cmd, "blink") == 0) {
                int count = msg.value;
                int delay_ms = (msg.value2 > 0) ? msg.value2 : 200;

                if (count < 0) count = 0;
                if (count > 20) count = 20;

                for (int i = 0; i < count; i++) {
                    Onboard_LED_control(true);
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    Onboard_LED_control(false);
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                }
                if (!text_message) mesh_zigbee_send_text("OK");
            }
            else if (strcmp(msg.cmd, "on") == 0) {
                if (!text_message) mesh_zigbee_send_text("GOT ON OK");
            }
            else if (strcmp(msg.cmd, "off") == 0) {
                if (!text_message) mesh_zigbee_send_text("GOT OFF OK");
            }
            else if (strcmp(msg.cmd, "SetupSerialBridge") == 0) {
                ESP_ERROR_CHECK(serial_bridge_init());
                mesh_zigbee_send_text("SETUP SERIAL OK");
            }
            // --- Dispatch Layer ---
            else if (pwm_driver_is_cmd(msg.cmd))   pwm_driver_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (motor_driver_is_cmd(msg.cmd)) motor_driver_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (gpio_handler_is_cmd(msg.cmd)) gpio_handler_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (neopixel_is_cmd(msg.cmd))     neopixel_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (BLDC_driver_is_cmd(msg.cmd))  BLDC_driver_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (adc_is_cmd(msg.cmd))          adc_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (stepper_is_cmd(msg.cmd))      stepper_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (proximity_is_cmd(msg.cmd))    proximity_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (vl53l1x_is_cmd(msg.cmd))      vl53l1x_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            else if (I2Craw_is_cmd(msg.cmd))       I2Craw_execute_cmd(msg.cmd, msg.value, msg.value2, text_message);
            // ----------------------
            else {
                if (!text_message) mesh_zigbee_send_text("INVALID");
            }
        }
    }
}

void app_main(void)
{
    ledc_manager_init();

    light_driver_init(LIGHT_DEFAULT_OFF);

    ESP_ERROR_CHECK(mesh_serial_init());
    ESP_ERROR_CHECK(mesh_zigbee_init());
    UpstreamQ_Init();

    xTaskCreate(app_msg_task, "app_msg_task", 3072, NULL, 5, NULL);
    xTaskCreate(serial_console_task, "serial_console_task", 3072, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
}