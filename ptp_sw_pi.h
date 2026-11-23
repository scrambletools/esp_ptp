#pragma once
#include <stdint.h>

/* The software clock's servo for a station's beacon-carrier time (AVB
 * Wireless Mode B). Its offsets come from the beacon element's min-gap
 * envelope in ptp_wifi.c, already filtered, so the loop can be damped:
 * proportional gain 2 zeta wn and integral gain wn^2, per second, with
 * wn 0.05 rad/s and zeta 0.8, scaled by the actual interval between
 * observations. The per-observation gains it replaces (offset / 2000 and
 * offset / 8000, at about one element a second) gave wn 0.011 rad/s and
 * zeta 0.02: after each association the clock swung +-2.5 ms on a 9.5 min
 * period and lost only about a tenth of the swing per cycle.
 * Output is the absolute trim in ppb. */
#define PTP_SW_PI_LIMIT 512000
#define PTP_SW_PI_KP_MICRO 80000 /* proportional gain, 1e-6 per s */
#define PTP_SW_PI_KI_MICRO 2500  /* integral gain, 1e-6 per s^2 */
#define PTP_SW_PI_OFFSET_MAX INT64_C(100000000) /* larger ones jump */

static inline int32_t ptp_sw_pi_step(int64_t *integral_q16, int64_t offset_ns,
                                    int64_t interval_ns)
{
    if (interval_ns < 10000000) interval_ns = 10000000;
    if (interval_ns > INT64_C(4000000000)) interval_ns = INT64_C(4000000000);
    if (offset_ns > PTP_SW_PI_OFFSET_MAX) offset_ns = PTP_SW_PI_OFFSET_MAX;
    if (offset_ns < -PTP_SW_PI_OFFSET_MAX) offset_ns = -PTP_SW_PI_OFFSET_MAX;
    int64_t proportional = offset_ns * PTP_SW_PI_KP_MICRO / 1000000;
    /* The integral's increment, ki x offset x interval, in 1e-12 ppb:
     * |offset x KI| <= 2.5e11 and the interval in us <= 4e6 stay under
     * 2^63, as does the Q16 conversion after dividing by 1e6 first. */
    int64_t scaled = offset_ns * PTP_SW_PI_KI_MICRO * (interval_ns / 1000);
    int64_t increment = (scaled / 1000000) * 65536 / 1000000;
    int64_t limit_q16 = (int64_t)PTP_SW_PI_LIMIT * 65536;
    int64_t candidate = *integral_q16 + increment;
    if (candidate > limit_q16) candidate = limit_q16;
    if (candidate < -limit_q16) candidate = -limit_q16;
    int64_t target = proportional + candidate / 65536;
    /* Hold the integral while the output is pinned in the offset's
     * direction, so a long slew does not wind it up. */
    if (!((target > PTP_SW_PI_LIMIT && offset_ns > 0) ||
          (target < -PTP_SW_PI_LIMIT && offset_ns < 0))) *integral_q16 = candidate;
    target = proportional + *integral_q16 / 65536;
    if (target > PTP_SW_PI_LIMIT) target = PTP_SW_PI_LIMIT;
    if (target < -PTP_SW_PI_LIMIT) target = -PTP_SW_PI_LIMIT;
    return (int32_t)target;
}
