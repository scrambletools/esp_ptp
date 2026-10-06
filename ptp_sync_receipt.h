#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Daemon-owned receipt state, measured on esp_timer's monotonic time base. */
typedef struct {
    int64_t received_us, interval_us;
    bool armed;
} ptp_sync_receipt_t;

static inline int64_t ptp_sync_receipt_interval(int8_t logarithm)
{
    if (logarithm < -24 || logarithm > 24) return 0;
    int64_t interval = INT64_C(3000000);
    if (logarithm >= 0) return interval << logarithm;
    int64_t divisor = INT64_C(1) << -logarithm;
    return (interval + divisor - 1) / divisor;
}

static inline void ptp_sync_receipt_start(ptp_sync_receipt_t *state,
    int64_t now_us, int64_t initial_interval_us)
{
    *state = (ptp_sync_receipt_t){.received_us = now_us,
        .interval_us = initial_interval_us,
        .armed = now_us >= 0 && initial_interval_us > 0};
}

static inline bool ptp_sync_receipt_current(const ptp_sync_receipt_t *state,
    int64_t now_us)
{
    return state->armed && now_us >= state->received_us &&
        now_us - state->received_us < state->interval_us;
}

static inline bool ptp_sync_receipt_observe(ptp_sync_receipt_t *state,
    int64_t received_us, int8_t logarithm, int64_t now_us)
{
    int64_t interval = ptp_sync_receipt_interval(logarithm);
    if (!interval || received_us < 0 || now_us < received_us ||
        now_us - received_us >= interval ||
        (state->armed && received_us < state->received_us)) return false;
    ptp_sync_receipt_start(state, received_us, interval);
    return true;
}

static inline int ptp_sync_receipt_wait_ms(const ptp_sync_receipt_t *state,
    int64_t now_us, int limit_ms)
{
    if (limit_ms <= 0 || !ptp_sync_receipt_current(state, now_us)) return 0;
    int64_t remaining = state->interval_us - (now_us - state->received_us);
    int64_t milliseconds = remaining / 1000 + (remaining % 1000 != 0);
    return milliseconds < limit_ms ? (int)milliseconds : limit_ms;
}
