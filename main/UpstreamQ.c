/*
 * UpstreamQ.c - Asynchronous Upstream Message Throttler & Queue
 *
 * Purpose:
 *   Provides a decoupled, thread-safe FreeRTOS message buffering and throttling service
 *   (s_msg_queue) for End Device sensor and status tasks. Allows sensor comparator tasks
 *   to post events non-blockingly via SendTheMessage() without stalling on wireless transmission,
 *   exerting automatic backpressure if the mesh TX queue is momentarily congested.
 *
 * Supported Commands:
 *   None (Internal Subsystem/Driver)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "mesh_zigbee.h"
#include "UpstreamQ.h"

#define MSG_BUFFER_LEN 10
#define MSG_MAX_SIZE 64

static const char *TAG = "UpstreamQ";
static QueueHandle_t s_msg_queue = NULL;

static void throttle_task(void *arg)
{
    char msg_buf[MSG_MAX_SIZE];

    while (1)
    {
        // Block indefinitely until a message is placed in the queue
        if (xQueueReceive(s_msg_queue, msg_buf, portMAX_DELAY) == pdTRUE)
        {
            // Forward to the mesh TX queue; if mesh TX queue is temporarily full,
            // retry with short backoff so s_msg_queue exerts backpressure rather than dropping.
            int retries = 0;
            while (!mesh_zigbee_send_text(msg_buf) && retries < 50)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                retries++;
            }
        }
    }
}

void UpstreamQ_Init(void)
{
    if (s_msg_queue == NULL)
    {
        s_msg_queue = xQueueCreate(MSG_BUFFER_LEN, MSG_MAX_SIZE);
        if (s_msg_queue != NULL)
        {
            xTaskCreate(throttle_task, "throttle_task", 2048, NULL, 5, NULL);
            ESP_LOGI(TAG, "Throttler task and queue initialized.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to create upstream message queue.");
        }
    }
}

bool SendTheMessage(const char *msg)
{
    if (msg == NULL) return false;

    // Keep local print for debugging
    printf("Queuing to Coordinator: %s\n", msg);

    if (s_msg_queue != NULL)
    {
        char buf[MSG_MAX_SIZE];
        strncpy(buf, msg, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        // Enqueue the message. If the queue is full, return false so caller can retry latest state.
        if (xQueueSend(s_msg_queue, buf, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "Message buffer full! Deferred/Dropped: %s", buf);
            return false;
        }
        return true;
    }
    return false;
}
