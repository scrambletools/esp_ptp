#pragma once
#include "ptp_signaling.h"

typedef struct {
    ptp_capable_message_t message;
    uint32_t lifecycle;
    int64_t received_us, expires_us;
    bool valid;
} ptp_capable_receive_t;

/* Receipt timeout is nine advertised intervals, rounded up to microseconds. */
static inline bool ptp_capable_receive(ptp_capable_receive_t *state,
    const ptp_capable_message_t *message, const uint8_t neighbor[10],
    uint32_t lifecycle, int64_t now_us)
{
    if (!neighbor || memcmp(message->source_port, neighbor, 10) ||
        message->log_interval < -24 || message->log_interval > 24 || now_us < 0)
        return false;
    int64_t interval = 9000000;
    if (message->log_interval >= 0) interval <<= message->log_interval;
    else {
        int64_t divisor = INT64_C(1) << -message->log_interval;
        interval = (interval + divisor - 1) / divisor;
    }
    if (now_us > INT64_MAX - interval) return false;
    *state = (ptp_capable_receive_t){.message = *message, .lifecycle = lifecycle,
        .received_us = now_us, .expires_us = now_us + interval, .valid = true};
    return true;
}

static inline bool ptp_neighbor_capable(const ptp_capable_receive_t *state,
    const uint8_t neighbor[10], uint32_t lifecycle, int64_t now_us)
{
    return state->valid && neighbor && state->lifecycle == lifecycle &&
        !memcmp(state->message.source_port, neighbor, 10) &&
        now_us >= state->received_us && now_us < state->expires_us;
}
