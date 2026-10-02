#include "serial_bridge.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mesh_zigbee.h"

static const char *TAG = "SERIAL_BRIDGE";

/*
 * XIAO ESP32-C6 hardware UART bridge settings.
 *
 * This file is intentionally separate from mesh_serial.c.
 * mesh_serial.c remains the USB debug/monitor console.
 *
 * UART bridge:
 *   Zigbee coordinator text -> GPIO21 / D3 TX @ 9600 baud
 *   GPIO22 / D4 RX @ 9600 baud -> Zigbee coordinator text
 *
 * Avoid GPIO16/GPIO17 here because this setup reports them as console UART pins.
 */



#define SERIAL_BRIDGE_UART_NUM     UART_NUM_1    // Peripheral 1
#define SERIAL_BRIDGE_TX_GPIO      GPIO_NUM_17   // Standard TX pin
#define SERIAL_BRIDGE_RX_GPIO      GPIO_NUM_16   // Standard RX pin
#define SERIAL_BRIDGE_BAUD_RATE    115200          // Matches your Arduino


// these pin assignments conflicted with i2c.
//#define SERIAL_BRIDGE_UART_NUM     UART_NUM_1
//#define SERIAL_BRIDGE_TX_GPIO      GPIO_NUM_21   // XIAO D3
//#define SERIAL_BRIDGE_RX_GPIO      GPIO_NUM_22   // XIAO D4
//#define SERIAL_BRIDGE_BAUD_RATE    115200

/*
 * ESP-IDF UART driver requires a valid RX buffer.
 * TX ring buffer is not required for these short bridge messages.
 */
#define SERIAL_BRIDGE_RX_BUF_SIZE  256
#define SERIAL_BRIDGE_TX_BUF_SIZE  0

#define SERIAL_BRIDGE_LINE_LEN     MESH_LINE_LEN

static void serial_bridge_rx_task(void *arg)
{
    char line[SERIAL_BRIDGE_LINE_LEN] = {0};
    size_t pos = 0;

    ESP_LOGI(TAG, "Hardware UART bridge RX task started on GPIO%d", SERIAL_BRIDGE_RX_GPIO);

    while (1) {
        uint8_t ch = 0;
        int n = uart_read_bytes(SERIAL_BRIDGE_UART_NUM, &ch, 1, pdMS_TO_TICKS(20));

        if (n != 1) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (pos == 0) {
                continue;
            }

            line[pos] = '\0';

            if (!mesh_zigbee_send_text(line)) {
                ESP_LOGW(TAG, "UART RX -> Zigbee queue full, dropping: %s", line);
            } else {
                ESP_LOGI(TAG, "UART RX -> Zigbee queued: %s", line);
            }

            pos = 0;
            memset(line, 0, sizeof(line));
        } else if (ch == 8 || ch == 127) {
            if (pos > 0) {
                pos--;
                line[pos] = '\0';
            }
        } else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)ch;
        } else {
            ESP_LOGW(TAG, "UART RX line too long, dropping");
            pos = 0;
            memset(line, 0, sizeof(line));
        }
    }
}

static bool s_bridge_initialized = false;

esp_err_t serial_bridge_init(void)
{
    if (s_bridge_initialized) {
        ESP_LOGI(TAG, "Hardware UART bridge already initialized");
        return ESP_OK;
    }

    uart_config_t cfg = {
        .baud_rate = SERIAL_BRIDGE_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(SERIAL_BRIDGE_UART_NUM,
                                            SERIAL_BRIDGE_RX_BUF_SIZE,
                                            SERIAL_BRIDGE_TX_BUF_SIZE,
                                            0,
                                            NULL,
                                            0),
                        TAG,
                        "uart_driver_install failed");

    ESP_RETURN_ON_ERROR(uart_param_config(SERIAL_BRIDGE_UART_NUM, &cfg),
                        TAG,
                        "uart_param_config failed");

    ESP_RETURN_ON_ERROR(uart_set_pin(SERIAL_BRIDGE_UART_NUM,
                                     SERIAL_BRIDGE_TX_GPIO,
                                     SERIAL_BRIDGE_RX_GPIO,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG,
                        "uart_set_pin failed");

    xTaskCreate(serial_bridge_rx_task,
                "serial_bridge_rx",
                3072,
                NULL,
                5,
                NULL);

    s_bridge_initialized = true;

    ESP_LOGI(TAG, "Hardware UART bridge ready: UART%d TX GPIO%d RX GPIO%d @ %d baud",
             SERIAL_BRIDGE_UART_NUM,
             SERIAL_BRIDGE_TX_GPIO,
             SERIAL_BRIDGE_RX_GPIO,
             SERIAL_BRIDGE_BAUD_RATE);

    return ESP_OK;
}

void serial_bridge_write(const char *s)
{
    if (!s || !s_bridge_initialized) {
        return;
    }

    uart_write_bytes(SERIAL_BRIDGE_UART_NUM, s, strlen(s));
}

void serial_bridge_write_line(const char *s)
{
    if (!s) {
        return;
    }

    serial_bridge_write(s);
    serial_bridge_write("\r\n");
}
