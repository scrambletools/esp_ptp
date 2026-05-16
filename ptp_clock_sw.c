/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Software clock backend for ptpd_now() on chips without IEEE 1588
 * hardware. See ptp_clock_sw.h for design notes.
 *
 * Time model:
 *   t_ptp_ns(local_us) = anchor_ptp_ns
 *                        + (local_us - anchor_local_us) * 1000
 *                        + (local_us - anchor_local_us) * rate_ppb / 1000000
 *
 * Re-anchoring on every settime/adjtime call keeps extrapolation
 * window short and arithmetic in 64-bit safely.
 *
 * Reads and writes happen on the daemon task, which is the same
 * thread, so no locking is required. If a higher-priority caller
 * ever needs to read the clock, mark the anchor pair atomic and use
 * a seqlock — not needed today.
 */

#include "ptp_clock_sw.h"

#include <errno.h>
#include <stddef.h>

#include "esp_timer.h"

/* Forward declaration of the indirection installer in ptpd.c. */
typedef int (*ptpd_sw_clock_now_fn)(struct timespec *ts);
extern void ptpd_set_sw_clock_now(ptpd_sw_clock_now_fn fn);

static int64_t s_anchor_local_us = 0;
static int64_t s_anchor_ptp_ns = 0;
static int32_t s_rate_ppb = 0;
static bool s_initialized = false;

static int64_t sw_now_ns(void)
{
    int64_t local_us = esp_timer_get_time();
    int64_t delta_us = local_us - s_anchor_local_us;
    int64_t delta_ns = delta_us * 1000;
    /* Rate correction: rate_ppb describes how much faster the
     * disciplined clock should advance vs. esp_timer. 1 ppb = 1 ns
     * extra per 1e9 ns = 1 ns extra per 1e6 us, so:
     *   correction_ns = delta_us * rate_ppb / 1000000
     * For ±1000 ppm (±1e6 ppb) and a typical re-anchor window of
     * <1 s, the correction stays well within int64_t range. */
    delta_ns += (delta_us * (int64_t)s_rate_ppb) / 1000000;
    return s_anchor_ptp_ns + delta_ns;
}

static void reanchor_to(int64_t ptp_ns)
{
    s_anchor_local_us = esp_timer_get_time();
    s_anchor_ptp_ns = ptp_ns;
}

static int sw_now_timespec(struct timespec *ts)
{
    if (!ts) {
        errno = EFAULT;
        return -1;
    }
    int64_t ns = sw_now_ns();
    ts->tv_sec = (time_t)(ns / 1000000000LL);
    ts->tv_nsec = (long)(ns % 1000000000LL);
    if (ts->tv_nsec < 0) {
        ts->tv_sec -= 1;
        ts->tv_nsec += 1000000000LL;
    }
    return 0;
}

int ptp_clock_sw_init(const struct timespec *initial_ts)
{
    int64_t initial_ns = 0;
    if (initial_ts) {
        initial_ns = (int64_t)initial_ts->tv_sec * 1000000000LL
                   + (int64_t)initial_ts->tv_nsec;
    }
    s_rate_ppb = 0;
    reanchor_to(initial_ns);
    s_initialized = true;
    ptpd_set_sw_clock_now(sw_now_timespec);
    return 0;
}

int ptp_clock_sw_now(struct timespec *ts)
{
    if (!s_initialized) {
        errno = EINVAL;
        return -1;
    }
    return sw_now_timespec(ts);
}

int ptp_clock_sw_settime(const struct timespec *ts)
{
    if (!s_initialized || !ts) {
        errno = EINVAL;
        return -1;
    }
    int64_t ns = (int64_t)ts->tv_sec * 1000000000LL + (int64_t)ts->tv_nsec;
    reanchor_to(ns);
    return 0;
}

int ptp_clock_sw_adjtime_offset(int64_t delta_ns)
{
    if (!s_initialized) {
        errno = EINVAL;
        return -1;
    }
    /* Re-anchor at the current time, then apply the delta to the
     * anchor so the next read sees the slewed value without picking
     * up a synthetic rate spike. */
    s_anchor_ptp_ns = sw_now_ns() + delta_ns;
    s_anchor_local_us = esp_timer_get_time();
    return 0;
}

int ptp_clock_sw_adjtime_rate(int32_t rate_ppb)
{
    if (!s_initialized) {
        errno = EINVAL;
        return -1;
    }
    /* RELATIVE adjustment to current rate, matching IDF's
     * clock_adjtime(CLOCK_PTP_SYSTEM, ADJ_FREQUENCY=ppb) semantics on
     * EMAC PTP (emac_hal_ptp_adj_freq multiplies current addend by
     * (1 + ppb/1e9)). The servo's freq_ppb formula relies on this
     * relative interpretation — passing it through as absolute makes
     * each call clobber the integrator and the loop oscillates. */
    int64_t now = sw_now_ns();
    int64_t cur = s_rate_ppb;
    int64_t delta = cur * (int64_t)rate_ppb / 1000000000LL + (int64_t)rate_ppb;
    int64_t new_rate = cur + delta;
    if (new_rate > 100000000) new_rate = 100000000;
    if (new_rate < -100000000) new_rate = -100000000;
    s_rate_ppb = (int32_t)new_rate;
    reanchor_to(now);
    return 0;
}
