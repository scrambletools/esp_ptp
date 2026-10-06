#pragma once
#include <stdint.h>

#define PTP_WIRED_TRIM_LIMIT 512000

/* Natural frequency 2 rad/s, reduced for long Sync intervals. Integral
 * gain includes the actual observation interval. Output is absolute ppb. */
static inline int32_t ptp_wired_pi_step(int64_t *integral_q16, int64_t offset_ns,
                                       int64_t interval_ns)
{
    if (interval_ns < 1000000) interval_ns = 1000000;
    if (interval_ns > INT64_C(4000000000)) interval_ns = INT64_C(4000000000);
    if (offset_ns > INT64_C(1000000000)) offset_ns = INT64_C(1000000000);
    if (offset_ns < -INT64_C(1000000000)) offset_ns = -INT64_C(1000000000);
    int64_t proportional, numerator, denominator;
    if (interval_ns <= 250000000) {
        proportional = offset_ns * 4;
        numerator = offset_ns * interval_ns;
        denominator = 250000000;
    } else {
        proportional = offset_ns * INT64_C(1000000000) / interval_ns;
        numerator = offset_ns * INT64_C(250000000);
        denominator = interval_ns;
    }
    int64_t increment = (numerator / denominator) * 65536 +
        (numerator % denominator) * 65536 / denominator;
    int64_t limit_q16 = (int64_t)PTP_WIRED_TRIM_LIMIT * 65536;
    int64_t candidate = *integral_q16 + increment;
    if (candidate > limit_q16) candidate = limit_q16;
    if (candidate < -limit_q16) candidate = -limit_q16;
    int64_t target = proportional + candidate / 65536;
    if (!((target > PTP_WIRED_TRIM_LIMIT && offset_ns > 0) ||
          (target < -PTP_WIRED_TRIM_LIMIT && offset_ns < 0))) *integral_q16 = candidate;
    target = proportional + *integral_q16 / 65536;
    if (target > PTP_WIRED_TRIM_LIMIT) target = PTP_WIRED_TRIM_LIMIT;
    if (target < -PTP_WIRED_TRIM_LIMIT) target = -PTP_WIRED_TRIM_LIMIT;
    return (int32_t)target;
}

static inline int32_t ptp_trim_relative_delta(int32_t previous, int32_t target)
{
    return (int32_t)(((int64_t)target - previous) * INT64_C(1000000000) /
                     (INT64_C(1000000000) + previous));
}
