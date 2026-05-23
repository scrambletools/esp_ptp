/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 * SPDX-FileContributor: 2024 Espressif Systems (Shanghai) CO LTD
 */

#ifndef __APPS_INCLUDE_NETUTILS_PTPD_H
#define __APPS_INCLUDE_NETUTILS_PTPD_H

#include <time.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifndef FAR
#define FAR
#endif

typedef enum
  {
    ptp_profile_standard = 0,
    ptp_profile_gptp = 1,
  } ptp_profile_e;

typedef enum
  {
    ptp_port_medium_eth_hwts = 0,
    ptp_port_medium_wifi_ftm = 1,
  } ptp_port_medium_e;

typedef enum
  {
    ptp_port_host_if_emac  = 0,
    ptp_port_host_if_ahb   = 1,
    ptp_port_host_if_sdio  = 2,
    ptp_port_host_if_spi   = 3,
    ptp_port_host_if_usb   = 4,
    ptp_port_host_if_other = 5,
  } ptp_port_host_if_e;

/* BRIDGED on any port implies the entity is in bridge mode. */
typedef enum
  {
    ptp_port_type_primary  = 0,
    ptp_port_type_failover = 1,
    ptp_port_type_bridged  = 2,
  } ptp_port_type_e;

/* Values match IDF's WIFI_IF_STA=0 / WIFI_IF_AP=1 for direct casting. */
typedef enum
  {
    ptp_port_wifi_mode_sta  = 0,
    ptp_port_wifi_mode_ap   = 1,
    ptp_port_wifi_mode_none = 0xFF,
  } ptp_port_wifi_mode_e;

typedef struct
  {
    uint8_t id[8];     /* Clock identity */
    int utcoffset;     /* Offset between clock time and UTC time (seconds) */
    int priority1;     /* Main priority field */
    int clockclass;    /* Clock class (IEEE-1588, lower is better) */
    int accuracy;      /* Clock accuracy (IEEE-1588, lower is better) */
    int variance;      /* Clock variance (IEEE-1588, lower is better) */
    int priority2;     /* Secondary priority field */
    uint8_t btc_id[8];  /* BTC clock identity */
    int stepsremoved;  /* How many steps from BTC clock */
    int timesource;    /* Type of time source (IEEE-1588) */
  } clock_info_s;

/* PTPD status information structure */

struct ptpd_status_s
{
  /* Active PTP profile. */

  ptp_profile_e ptp_profile;

  /* AVB Lite Endpoint Declaration TLV detected on the Pdelay channel
   * (profiles/avb_lite.md §2.1/§2.2). True when the immediate Pdelay peer
   * is also an endpoint, indicating no AVB-aware bridge sits between us. */

  bool peer_is_endpoint;

  /* Is there a valid remote clock source active? */

  bool clock_source_valid;

  /* Information about selected best clock source */

  clock_info_s clock_source_info;

  /* Information about local clock */

  clock_info_s own_identity_info;

  /* When was clock last updated or adjusted (CLOCK_REALTIME).
   * Matches last_received_sync but in different clock.
   */

  struct timespec last_clock_update;

  /* Details of clock adjustment made at last_clock_update */

  int64_t last_delta_ns;     /* Latest measured clock error */
  int64_t last_adjtime_ns;   /* Previously applied adjtime() offset */

  /* Averaged clock drift estimate (parts per billion).
   * Positive means remote clock runs faster than local clock before
   * adjustment.
   */

  long drift_ppb;

  /* Averaged delay */

  long path_delay_ns;
  long peer_delay_ns;

  /* Timestamps of latest received packets (CLOCK_MONOTONIC) */

  struct timespec last_received_multicast; /* Any multicast packet */
  struct timespec last_received_announce;  /* Announce from any server */
  struct timespec last_received_sync;      /* Sync from selected source */

  /* Timestamps of latest transmitted packets (CLOCK_MONOTONIC) */

  struct timespec last_transmitted_sync;
  struct timespec last_transmitted_announce;
  struct timespec last_transmitted_delayresp;
  struct timespec last_transmitted_delayreq;
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Start the PTP daemon on `interface`. Returns task ID or -errno. */
int ptpd_start(FAR const char *interface);

/* Multi-port variant: bootstraps the daemon on first call (only an
 * eth_hwts port can bootstrap; wifi_ftm attaches to an existing
 * daemon) and configures port_index with the given medium. */
int ptpd_start_port(int port_index,
                    FAR const char *interface,
                    ptp_port_medium_e medium);

/* Push an externally-measured peer-delay sample (e.g. FTM-derived
 * on a wifi_ftm STA port) into the running averager. */
int ptpd_inject_peer_delay(int port_index, int64_t peer_delay_ns);

/* Push a marshalled FollowUpInformation TLV (IEEE 802.1AS-2020 §12.7)
 * for ports whose Sync transport is out-of-band (e.g. beacon Vendor IE).
 * Ownership of follow_up_info stays with the caller. */
int ptpd_inject_sync(int port_index,
                     FAR const uint8_t *follow_up_info,
                     size_t len);

/* Push (remote_ns, local_ns) directly into the servo, bypassing the
 * §12.7 byte parser. For FTM-derived sync where the caller has already
 * resolved BTC time at TX and local time at RX. The pair must bracket
 * the same wall-clock moment modulo peer_delay_ns (subtracted by the
 * daemon). */
int ptpd_inject_sync_pair(int port_index,
                          int64_t remote_ns,
                          int64_t local_ns);

/* Register a callback fired whenever the daemon would emit a gPTP
 * Sync/Follow_Up on port_index. The callback carries the marshalled
 * FollowUpInformation TLV to the wire (e.g. beacon Vendor IE update).
 * One callback per port; NULL clears. */
typedef void (*ptpd_sync_egress_cb_t)(int port_index,
                                      FAR const uint8_t *follow_up_info,
                                      size_t len,
                                      FAR void *ctx);

int ptpd_register_sync_egress_cb(int port_index,
                                 ptpd_sync_egress_cb_t cb,
                                 FAR void *ctx);

/* Read the PTP-disciplined system time. Equivalent to clock_gettime
 * (CLOCK_PTP_SYSTEM) on platforms with EMAC 1588 hardware; sources
 * from the software clock (esp_timer-backed) on platforms without. */
int ptpd_now(FAR struct timespec *ts);

/* Change the active PTP profile at runtime. */
int ptpd_set_profile(int pid, ptp_profile_e profile);

/* Query daemon status. Threads with priority below
 * CONFIG_NETUTILS_PTPD_SERVERPRIO may time out if higher-priority
 * threads request status concurrently. */
int ptpd_status(int pid, FAR struct ptpd_status_s *status);

int ptpd_stop(int pid);

/* True if port_index is enabled and its link/association is up.
 * Defaults to true on a freshly bootstrapped port so the first TX
 * cycle isn't dropped before the first link event fires. */
bool ptpd_port_link_up(int port_index);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_PTPD_H */
