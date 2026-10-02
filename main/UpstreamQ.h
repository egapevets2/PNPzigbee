#ifndef UPSTREAM_Q_H
#define UPSTREAM_Q_H

// Initialize the upstream message queue and throttling task.
// Call this once during your main boot sequence before starting tasks.
void UpstreamQ_Init(void);

// Queue a message to be sent to the coordinator.
void SendTheMessage(const char *msg);

#endif // UPSTREAM_Q_H