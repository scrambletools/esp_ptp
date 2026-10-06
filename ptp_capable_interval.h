#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Local supported rate range, at most eight capability messages per second. */
typedef struct {
    int8_t log_interval;
    uint8_t slowdown_remaining;
    int64_t old_interval_us;
} ptp_capable_interval_t;

static inline int64_t ptp_capable_interval_us(int8_t logarithm)
{
    if (logarithm < -3 || logarithm > 24) return 0;
    return logarithm < 0 ? INT64_C(1000000) >> -logarithm :
                          INT64_C(1000000) << logarithm;
}

static inline int64_t ptp_capable_interval_period(const ptp_capable_interval_t *state)
{
    if (state->log_interval == 127) return 0;
    return state->slowdown_remaining ? state->old_interval_us :
                                      ptp_capable_interval_us(state->log_interval);
}

/* Reserved values do not mutate state. Requests faster than the supported
 * maximum rate select the closest longer supported interval, and slower
 * requests up to 24 are all supported. */
static inline bool ptp_capable_interval_request(ptp_capable_interval_t *state, int8_t request)
{
    if (request == -128) return true;
    if (request == 126) request = 0;
    if (request >= -24 && request < -3) request = -3;
    if (request != 127 && !ptp_capable_interval_us(request)) return false;
    if (request == state->log_interval) return true;
    int64_t previous = ptp_capable_interval_period(state);
    int64_t next = ptp_capable_interval_us(request);
    state->log_interval = request;
    state->slowdown_remaining = previous && next > previous ? 9 : 0;
    state->old_interval_us = previous;
    return true;
}

/* Count only accepted transmissions of the currently advertised interval. */
static inline void ptp_capable_interval_sent(ptp_capable_interval_t *state,
                                            int8_t advertised_interval, bool accepted)
{
    if (accepted && state->log_interval == advertised_interval && state->slowdown_remaining)
        --state->slowdown_remaining;
}
