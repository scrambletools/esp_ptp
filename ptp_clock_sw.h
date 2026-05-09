/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * esp_ptp software clock backend.
 *
 * For chips without IEEE 1588 hardware (e.g. ESP32-C6) where
 * CLOCK_PTP_SYSTEM is not pluggable from application code, the daemon
 * disciplines a software clock backed by esp_timer_get_time() and a
 * pair of (offset_ns, rate_ppb) tunables. ptpd_now() routes through
 * this backend when ptp_clock_sw_init() has been called.
 *
 * On platforms with EMAC 1588 (ESP32-P4) this file is unused; the
 * legacy esp_eth_clock path remains the source of truth.
 *
 * esp_ptp must stand alone — every identifier here is in the
 * ptp_clock_sw_* namespace, not avb_*. (esp_avb is the eventual
 * primary consumer; that context is captured here in comments only.)
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
