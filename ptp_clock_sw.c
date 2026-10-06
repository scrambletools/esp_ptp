/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Software PTP clock with a coherent affine anchor and relative rate control.
 * Capture each update's local time once so rate changes preserve continuity.
 */
#include "ptp.h"
#include "ptp_clock_affine.h"
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#if CONFIG_IDF_TARGET_ESP32C6
#include "esp_timer_impl.h"
#endif

typedef int (*ptpd_sw_clock_now_fn)(struct timespec *ts);
extern void ptpd_set_sw_clock_now(ptpd_sw_clock_now_fn fn);

static ptp_clock_affine_t s_clock;
static bool s_initialized;
static portMUX_TYPE s_clock_lock = portMUX_INITIALIZER_UNLOCKED;

bool ptp_clock_sw_snapshot(ptp_clock_affine_t *snapshot)
{
    if (!snapshot) return false;
    portENTER_CRITICAL(&s_clock_lock);
    bool initialized = s_initialized;
    *snapshot = s_clock;
    portEXIT_CRITICAL(&s_clock_lock);
    return initialized;
}

int64_t ptp_clock_local_ns(void)
{
#if CONFIG_IDF_TARGET_ESP32C6
    /* C6 SYSTIMER uses the 40 MHz crystal divided by 2.5, or 62.5 ns/tick. */
    uint64_t ticks = esp_timer_impl_get_counter_reg();
    return (int64_t)(ticks / 2 * 125 + (ticks & 1) * 62);
#else
    return esp_timer_get_time() * 1000;
#endif
}

static int sw_now_timespec(struct timespec *ts)
{
    if (!ts) { errno = EFAULT; return -1; }
    ptp_clock_affine_t snapshot;
    int64_t local_ns;
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL;
        return -1;
    }
    snapshot = s_clock;
    local_ns = ptp_clock_local_ns();
    portEXIT_CRITICAL(&s_clock_lock);
    int64_t now_ns = ptp_clock_affine_now(&snapshot, local_ns);
    ts->tv_sec = (time_t)(now_ns / 1000000000LL);
    ts->tv_nsec = (long)(now_ns % 1000000000LL);
    if (ts->tv_nsec < 0) { --ts->tv_sec; ts->tv_nsec += 1000000000L; }
    return 0;
}

int ptp_clock_sw_init(const struct timespec *initial_ts)
{
    int64_t initial_ns = initial_ts ?
        (int64_t)initial_ts->tv_sec * 1000000000LL + initial_ts->tv_nsec : 0;
    portENTER_CRITICAL(&s_clock_lock);
    s_clock = (ptp_clock_affine_t){.local_ns = ptp_clock_local_ns(),
                                  .ptp_ns = initial_ns};
    s_initialized = true;
    portEXIT_CRITICAL(&s_clock_lock);
    ptpd_set_sw_clock_now(sw_now_timespec);
    return 0;
}

int ptp_clock_sw_now(struct timespec *ts)
{
    return sw_now_timespec(ts);
}

int ptp_clock_sw_settime(const struct timespec *ts)
{
    if (!ts) { errno = EINVAL; return -1; }
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    s_clock.local_ns = ptp_clock_local_ns();
    s_clock.ptp_ns = (int64_t)ts->tv_sec * 1000000000LL + ts->tv_nsec;
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_adjtime_offset(int64_t delta_ns)
{
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    ptp_clock_affine_reanchor(&s_clock, ptp_clock_local_ns(), delta_ns);
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_adjtime_rate(int32_t rate_ppb)
{
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    ptp_clock_affine_adjust_rate(&s_clock, ptp_clock_local_ns(), rate_ppb);
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_discipline(int64_t local_anchor, int64_t reference_anchor,
    int32_t rate_ppb, bool step, int64_t *phase_error, int32_t *applied_rate)
{
    if (!phase_error || !applied_rate || local_anchor < 0 || reference_anchor < 0 ||
        reference_anchor > INT64_MAX - INT64_C(3000000000) ||
        rate_ppb < -200000 || rate_ppb > 200000) return -1;
    portENTER_CRITICAL(&s_clock_lock);
    int64_t now_ns = ptp_clock_local_ns();
    if (!s_initialized || now_ns < local_anchor || now_ns - local_anchor > INT64_C(2000000000)) {
        portEXIT_CRITICAL(&s_clock_lock);
        return -1;
    }
    ptp_clock_affine_discipline(&s_clock, now_ns, local_anchor, reference_anchor,
        rate_ppb, step, phase_error);
    *applied_rate = s_clock.rate_ppb;
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}
