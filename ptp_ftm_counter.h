#ifndef PTP_FTM_COUNTER_H
#define PTP_FTM_COUNTER_H

#include <stdbool.h>
#include <stdint.h>

/* Relative continuity only; the counter is not AP TSF. */
static inline bool ptp_ftm_counter_backward(uint64_t previous, uint64_t current)
{
    const uint64_t period = UINT64_C(1) << 48;
    const uint64_t tolerance = UINT64_C(1000000000000);
    if (previous >= period || current >= period) return false;
    uint64_t forward = (current - previous) & (period - 1);
    return forward > period / 2 && period - forward > tolerance;
}

#endif
