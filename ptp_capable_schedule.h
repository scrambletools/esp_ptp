#pragma once
#include "ptp_capable_interval.h"

/* One outstanding transport operation per association. */
typedef struct {
    ptp_capable_interval_t interval;
    int64_t next_us, queued_us;
    uint32_t serial, pending;
    uint16_t sequence;
    int8_t advertised_interval;
    bool active;
} ptp_capable_schedule_t;

static inline bool ptp_capable_schedule_request(ptp_capable_schedule_t *state,
                                                int8_t request)
{
    int8_t previous = state->interval.log_interval;
    if (!ptp_capable_interval_request(&state->interval, request)) return false;
    if (state->interval.log_interval != previous) {
        /* A late completion cannot count toward a replacement request. */
        state->pending = 0;
        state->active = false;
    }
    return true;
}

/* Preserve the deadline across small delays; skip fully missed periods. */
static inline uint32_t ptp_capable_schedule_begin(ptp_capable_schedule_t *state,
                                                 bool enabled, int64_t now_us)
{
    if (!enabled) {
        state->pending = 0;
        state->active = false;
        return 0;
    }
    int64_t period = ptp_capable_interval_period(&state->interval);
    if (!period || now_us < 0 || now_us > INT64_MAX - period) return 0;
    if (state->pending) {
        if (now_us < state->queued_us || now_us - state->queued_us < 1000000)
            return 0;
        /* The transport also rejects work aged one second or more. */
        state->pending = 0;
    }
    if (state->active && now_us < state->next_us) return 0;
    if (!state->active || now_us - state->next_us >= period)
        state->next_us = now_us + period;
    else
        state->next_us += period;
    state->queued_us = now_us;
    state->active = true;
    if (!++state->serial) ++state->serial;
    state->pending = state->serial;
    state->advertised_interval = state->interval.log_interval;
    ++state->sequence;
    return state->pending;
}

/* Transport acceptance is not a radio acknowledgement or delivery guarantee. */
static inline bool ptp_capable_schedule_finish(ptp_capable_schedule_t *state,
    uint32_t token, bool accepted, int64_t now_us)
{
    if (!token || state->pending != token) return false;
    state->pending = 0;
    if (now_us < state->queued_us || now_us - state->queued_us >= 1000000)
        return false;
    int64_t previous = ptp_capable_interval_period(&state->interval);
    ptp_capable_interval_sent(&state->interval, state->advertised_interval, accepted);
    int64_t period = ptp_capable_interval_period(&state->interval);
    if (period != previous && now_us <= INT64_MAX - period)
        state->next_us = now_us + period;
    return true;
}

/* Pending transport work is polled only after its next scheduled deadline. */
static inline int ptp_capable_schedule_wait_ms(const ptp_capable_schedule_t *state,
                                              int64_t now_us, int maximum_ms)
{
    if (maximum_ms <= 0 || now_us < 0 || !ptp_capable_interval_period(&state->interval))
        return maximum_ms;
    int64_t delay = state->active && state->next_us > now_us ? state->next_us - now_us : 0;
    if (state->pending && delay == 0 && now_us >= state->queued_us &&
        now_us - state->queued_us < 1000000) delay = 10000;
    int64_t milliseconds = delay / 1000 + (delay % 1000 != 0);
    return milliseconds < maximum_ms ? (int)milliseconds : maximum_ms;
}
