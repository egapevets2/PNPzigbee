/*
 * UpstreamQ.h - Asynchronous Upstream Message Throttler & Queue Interface
 *
 * Purpose:
 *   Provides thread-safe FreeRTOS message buffering (s_msg_queue) and immediate forwarding
 *   for asynchronous sensor/comparator events and status reports sent to the Coordinator.
 */
#ifndef UPSTREAM_Q_H
#define UPSTREAM_Q_H

#include <stdbool.h>

// Initialize the upstream message queue and throttling task.
// Call this once during your main boot sequence before starting tasks.
void UpstreamQ_Init(void);

// Queue a message to be sent to the coordinator.
// Returns true if enqueued, false if the queue was full.
bool SendTheMessage(const char *msg);

#endif // UPSTREAM_Q_H
