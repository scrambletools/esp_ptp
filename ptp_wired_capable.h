#pragma once
#include <string.h>
#include "ptp_signaling.h"
#include "ptp_capable_schedule.h"

typedef struct {
    ptp_capable_schedule_t transmit;
    uint32_t lifecycle;
    uint8_t identity[10];
    bool bound;
} ptp_wired_capable_t;

/* Lifecycle invalidation is independent of temporary rate-estimator validity. */
static inline void ptp_wired_capable_bind(ptp_wired_capable_t *state,
    uint32_t lifecycle, const uint8_t identity[10])
{
    if (state->lifecycle != lifecycle ||
        (identity && state->bound && memcmp(state->identity, identity, 10))) {
        uint32_t serial = state->transmit.serial;
        memset(state, 0, sizeof(*state));
        state->transmit.serial = serial;
        state->lifecycle = lifecycle;
    }
    if (identity) {
        memcpy(state->identity, identity, 10);
        state->bound = true;
    }
}

static inline bool ptp_wired_capable_request(ptp_wired_capable_t *state,
    uint32_t lifecycle, const ptp_capable_message_t *request)
{
    return state->bound && state->lifecycle == lifecycle &&
        !memcmp(state->identity, request->source_port, 10) &&
        ptp_capable_schedule_request(&state->transmit, request->log_interval);
}
