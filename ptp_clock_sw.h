/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Software clock backend used when CLOCK_PTP_SYSTEM is not pluggable
 * (e.g. ESP32-C6). Disciplines (offset_ns, rate_ppb) over
 * esp_timer_get_time(). ptpd_now() routes through this once
 * ptp_clock_sw_init() runs; on EMAC 1588 platforms it is unused.
 */

#ifndef ESP_PTP_CLOCK_SW_H
#define ESP_PTP_CLOCK_SW_H

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize and select the software clock as the backend for
 * ptpd_now(). Idempotent. After this call ptpd_now() returns the
 * disciplined software time instead of clock_gettime(CLOCK_PTP_SYSTEM).
 *
 * initial_ts may be NULL, in which case the clock starts at 0. */
int ptp_clock_sw_init(const struct timespec *initial_ts);

/* Read the current software time. */
int ptp_clock_sw_now(struct timespec *ts);

/* Step the software time to ts. Discontinuous — re-anchors immediately. */
int ptp_clock_sw_settime(const struct timespec *ts);

/* Slew the software time by delta_ns. Continuous — preserves rate. */
int ptp_clock_sw_adjtime_offset(int64_t delta_ns);

/* Set the rate offset, in parts-per-billion relative to the local
 * esp_timer. Positive ppb means the software clock runs faster than
 * the local oscillator. Range +/- 1e6 ppb (+/- 1000 ppm) is
 * comfortable; anything beyond ±1e8 is rejected. Re-anchors so the
 * change does not introduce a discontinuity. */
int ptp_clock_sw_adjtime_rate(int32_t rate_ppb);

#ifdef __cplusplus
}
#endif

#endif /* ESP_PTP_CLOCK_SW_H */
