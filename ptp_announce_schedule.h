#pragma once
#include <stdbool.h>
#include <stdint.h>

/* AP publication supplies the configured base rate; support that rate and slower. */
typedef struct {
    bool initialized, active;
    int8_t initial_log, log_interval, pending_log;
    uint8_t slowdown_remaining;
    uint16_t sequence;
    uint32_t revision, serial, pending, information_revision;
    int64_t old_period_us, next_us, queued_us;
} ptp_announce_schedule_t;

/* Retire copied work without reusing completion tokens or local sequences. */
static inline void ptp_announce_reset(ptp_announce_schedule_t *state)
{
    uint32_t revision = state->revision + 1;
    *state = (ptp_announce_schedule_t){
        .revision = revision ? revision : 1,
        .serial = state->serial,
        .sequence = state->sequence,
    };
}

static inline int64_t ptp_announce_period_us(int8_t logarithm)
{
    if (logarithm < -24 || logarithm > 24) return 0;
    if (logarithm >= 0) return INT64_C(1000000) << logarithm;
    uint32_t divisor = UINT32_C(1) << -logarithm;
    return (UINT32_C(1000000) + divisor - 1) / divisor;
}

static inline bool ptp_announce_initialize(ptp_announce_schedule_t *state, int8_t initial)
{
    if (!ptp_announce_period_us(initial)) return false;
    if (state->initialized) return state->initial_log == initial;
    state->initialized = true;
    state->initial_log = state->log_interval = initial;
    return true;
}

static inline int64_t ptp_announce_effective_period(const ptp_announce_schedule_t *state)
{
    if (state->log_interval == 127) return 0;
    return state->slowdown_remaining ? state->old_period_us :
                                      ptp_announce_period_us(state->log_interval);
}

/* Reserved values are ignored; unsupported faster rates select the closest longer rate. */
static inline bool ptp_announce_request(ptp_announce_schedule_t *state,
                                        int8_t initial, int8_t request)
{
    if (request != -128 && request != 126 && request != 127 &&
        (request < -24 || request > 24)) return false;
    if (!ptp_announce_initialize(state, initial)) return false;
    if (request == -128) return true;
    if (request == 126) request = initial;
    if (request != 127 && request < initial) request = initial;
    if (request == state->log_interval) return true;
    int64_t previous = ptp_announce_effective_period(state);
    int64_t next = ptp_announce_period_us(request);
    state->log_interval = request;
    state->old_period_us = previous;
    state->slowdown_remaining = previous && next > previous ? 3 : 0;
    state->pending = 0;
    state->active = false;
    if (!++state->revision) ++state->revision;
    return true;
}

static inline uint32_t ptp_announce_begin(ptp_announce_schedule_t *state,
    int8_t initial, uint32_t expected_revision, uint32_t information_revision,
    int64_t now_us, int8_t *advertised, uint16_t *sequence)
{
    if (!advertised || !sequence || state->revision != expected_revision ||
        !ptp_announce_initialize(state, initial)) return 0;
    int64_t period = ptp_announce_effective_period(state);
    if (!period || now_us < 0 || now_us > INT64_MAX - period) return 0;
    if (state->information_revision != information_revision) {
        state->information_revision = information_revision;
        state->pending = 0;
        state->active = false;
    }
    if (state->pending) {
        if (now_us < state->queued_us || now_us - state->queued_us < 1000000) return 0;
        state->pending = 0;
    }
    if (state->active && now_us < state->next_us) return 0;
    if (!state->active || now_us - state->next_us >= period) state->next_us = now_us + period;
    else state->next_us += period;
    state->active = true;
    state->queued_us = now_us;
    if (!++state->serial) ++state->serial;
    state->pending = state->serial;
    state->pending_log = state->log_interval;
    *advertised = state->log_interval;
    *sequence = ++state->sequence;
    return state->pending;
}

/* Count transport acceptance, not queueing or a radio-delivery acknowledgement. */
static inline bool ptp_announce_finish(ptp_announce_schedule_t *state,
    uint32_t token, bool accepted, int64_t now_us)
{
    if (!token || token != state->pending) return false;
    state->pending = 0;
    if (now_us < state->queued_us || now_us - state->queued_us >= 1000000) return false;
    if (accepted && state->pending_log == state->log_interval && state->slowdown_remaining) {
        if (!--state->slowdown_remaining) {
            int64_t period = ptp_announce_period_us(state->log_interval);
            if (now_us <= INT64_MAX - period) state->next_us = now_us + period;
        }
    }
    return true;
}
