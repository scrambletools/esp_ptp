#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t lifecycle;
    int64_t received_us;
    bool qualified;
} ptp_peer_capability_t;

/* Conservative three-interval expiry, independent of selected clock source. */
static inline bool ptp_peer_capable(const ptp_peer_capability_t *capability,
    uint32_t lifecycle, int64_t now_us, int64_t interval_us,
    bool enabled, bool link_up, bool gptp, unsigned domain, bool neighbor_capable)
{
    return enabled && link_up && gptp && (domain == 0 || neighbor_capable) &&
        capability->qualified && capability->lifecycle == lifecycle &&
        interval_us > 0 && interval_us <= INT64_MAX / 3 &&
        now_us >= capability->received_us &&
        now_us - capability->received_us < interval_us * 3;
}
