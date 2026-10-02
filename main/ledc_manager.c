#include "ledc_manager.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LEDC_MGR";

// ESP32-C6 specifically has 6 channels and 4 timers in LOW_SPEED_MODE
#define MAX_CHANNELS 6
#define MAX_TIMERS   4

static bool channel_in_use[MAX_CHANNELS];
static bool timer_in_use[MAX_TIMERS];
static int  timer_frequencies[MAX_TIMERS]; // Tracks the frequency assigned to each timer

void ledc_manager_init(void)
{
    memset(channel_in_use, 0, sizeof(channel_in_use));
    memset(timer_in_use, 0, sizeof(timer_in_use));
    memset(timer_frequencies, 0, sizeof(timer_frequencies));
    ESP_LOGI(TAG, "LEDC Resource Manager Initialized");
}

bool ledc_manager_alloc_channel(ledc_channel_t *out_channel)
{
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (!channel_in_use[i])
        {
            channel_in_use[i] = true;
            *out_channel = (ledc_channel_t)i;
            ESP_LOGI(TAG, "Allocated LEDC Channel %d", i);
            return true;
        }
    }
    ESP_LOGE(TAG, "FAILED to allocate LEDC Channel (All %d in use)", MAX_CHANNELS);
    return false;
}

bool ledc_manager_alloc_timer(int freq_hz, ledc_timer_t *out_timer)
{
    // First pass: Check if a timer is already running at this exact frequency
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        if (timer_in_use[i] && timer_frequencies[i] == freq_hz)
        {
            *out_timer = (ledc_timer_t)i;
            ESP_LOGI(TAG, "Shared existing LEDC Timer %d (%d Hz)", i, freq_hz);
            return true;
        }
    }

    // Second pass: Find a completely free timer
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        if (!timer_in_use[i])
        {
            timer_in_use[i] = true;
            timer_frequencies[i] = freq_hz;
            *out_timer = (ledc_timer_t)i;
            ESP_LOGI(TAG, "Allocated new LEDC Timer %d for %d Hz", i, freq_hz);
            return true;
        }
    }

    ESP_LOGE(TAG, "FAILED to allocate LEDC Timer (All %d in use by other frequencies)", MAX_TIMERS);
    return false;
}