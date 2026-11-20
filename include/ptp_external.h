/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * External time source: gPTP time measured some other way than this
 * daemon's own ports, such as a Wi-Fi coprocessor's FTM time engine,
 * given as pairs of this PTP clock and gPTP time at the same instant. It
 * can steer the hardware clock when the wired source is lost, so an
 * endpoint keeps gPTP time through a failover to Wi-Fi, and hands back
 * to the wired source when it returns. The servo is the wired one, its
 * integral shared, so the clock's rate carries across both ways.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PTP_EXTERNAL_OFF = 0,
  /* Measured against the clock and logged; never steers it. */
  PTP_EXTERNAL_SHADOW,
  /* Steers the clock while the wired source is lost and samples are
   * fresh; the wired source takes back over when valid again. */
  PTP_EXTERNAL_STANDBY,
  /* Steers the clock even while the wired source is valid; wired Sync is
   * then only measured (EXTERNAL lines show its offset). For tests. */
  PTP_EXTERNAL_FORCED,
} ptp_external_mode_t;

typedef struct {
  int64_t local_ns;     /* this daemon's PTP clock at the instant */
  int64_t reference_ns; /* gPTP time at the same instant */
  int64_t received_us;  /* esp_timer_get_time() when it was made */
} ptp_external_sample_t;

typedef struct {
  ptp_external_mode_t mode;
  bool driving;         /* it steers the clock now */
  bool fresh;           /* a sample in the last second */
  int64_t offset_ns;    /* the latest averaged offset, gPTP minus clock */
  uint32_t updates;     /* servo updates it made */
} ptp_external_status_t;

void ptpd_external_set_mode(ptp_external_mode_t mode);

/* Any task. Queued for the daemon; false when the queue is full. */
bool ptpd_external_observe(const ptp_external_sample_t *sample);

void ptpd_external_status(ptp_external_status_t *status);

#ifdef __cplusplus
}
#endif
