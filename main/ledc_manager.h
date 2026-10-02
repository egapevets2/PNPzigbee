#pragma once

#include <stdbool.h>
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the central LEDC resource manager.
 */
void ledc_manager_init(void);

/**
 * @brief Allocate a free LEDC channel.
 * * @param out_channel Pointer to store the allocated channel.
 * @return true if successful, false if all 6 channels are in use.
 */
bool ledc_manager_alloc_channel(ledc_channel_t *out_channel);

/**
 * @brief Allocate a timer. If a timer already exists with the requested 
 * frequency, it shares that timer to save hardware resources.
 * * @param freq_hz The requested frequency.
 * @param out_timer Pointer to store the allocated timer.
 * @return true if successful, false if all 4 timers are in use by other frequencies.
 */
bool ledc_manager_alloc_timer(int freq_hz, ledc_timer_t *out_timer);

#ifdef __cplusplus
}
#endif