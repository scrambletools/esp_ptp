/*
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2024 Espressif Systems (Shanghai) CO LTD
 */

/****************************************************************************
 * apps/include/netutils/ptpd.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_PTPD_H
#define __APPS_INCLUDE_NETUTILS_PTPD_H

// ESP_PTP
#include <time.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifndef FAR
#define FAR
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
  {
    ptp_profile_standard = 0,
    ptp_profile_gptp = 1,
  } ptp_profile_e;

/* Per-port medium and timing mechanism, named together because they
 * determine the on-wire protocol the daemon runs for this port.
 *
 *   eth_hwts — Ethernet medium with IEEE 1588 hardware timestamping
 *              and on-wire 802.1AS Sync / Follow_Up / Pdelay. The
 *              port owns an L2TAP socket bound to its interface and
 *              participates in the standard gPTP exchange.
 *
 *   wifi_ftm — Wi-Fi medium with peer-delay measured via IEEE
 *              802.11mc FTM and Sync transported in the SoftAP's
 *              802.11 Beacon Vendor IE. No L2TAP socket; the daemon
 *              dispatches FollowUpInformation through the registered
 *              sync_egress_cb on the AP side and accepts peer-delay
 *              samples via ptpd_inject_peer_delay() on the STA side.
 *              Whether this port measures FTM or just responds is
 *              derived from wifi_mode (STA initiates; AP responds). */

typedef enum
  {
    ptp_port_medium_eth_hwts = 0,
    ptp_port_medium_wifi_ftm = 1,
  } ptp_port_medium_e;

/* Port-host-interface taxonomy. How the port is physically attached to
 * the host SoC — determines timestamp accuracy, latency floor, and
 * per-port admission budget. EMAC is the only category with hardware
 * timestamping; the rest fall back to software timestamping with
 * progressively worse jitter as you move down the list. */

typedef enum
  {
    ptp_port_host_if_emac  = 0,   /* on-chip MAC, RMII/RGMII; HW TS */
    ptp_port_host_if_ahb   = 1,   /* on-chip peripheral on AHB (native Wi-Fi) */
    ptp_port_host_if_sdio  = 2,   /* external coprocessor over SDIO */
    ptp_port_host_if_spi   = 3,   /* external Eth/Wi-Fi chip over SPI */
    ptp_port_host_if_usb   = 4,   /* USB-attached */
    ptp_port_host_if_other = 5,
  } ptp_port_host_if_e;

/* Port type — the port's role in the gPTP / AVB topology. PRIMARY and
 * FAILOVER are endpoint-side; BRIDGED ports participate in an L2 bridge
 * with the other BRIDGED port(s) on the same entity. Any port being
 * BRIDGED implies the entity is in bridge mode. */

typedef enum
  {
    ptp_port_type_primary  = 0,
    ptp_port_type_failover = 1,
    ptp_port_type_bridged  = 2,
  } ptp_port_type_e;

/* Wi-Fi mode for ports with medium=wifi. Numeric values match IDF's
 * WIFI_IF_STA=0 / WIFI_IF_AP=1 so the field can be cast directly when
 * calling esp_wifi_internal_tx. NONE is for non-wifi ports. */

typedef enum
  {
    ptp_port_wifi_mode_sta  = 0,    /* joins external AP */
    ptp_port_wifi_mode_ap   = 1,    /* hosts SoftAP */
    ptp_port_wifi_mode_none = 0xFF, /* port is not wifi */
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ptpd_start
 *
 * Description:
 *   Start the PTP daemon and bind it to specified interface.
 *
 * Input Parameters:
 *   interface - Name of the network interface to bind to, e.g. "eth0"
 *
 * Returned Value:
 *   On success, the non-negative task ID of the PTP daemon is returned;
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_start(FAR const char *interface);

/****************************************************************************
 * Name: ptpd_start_port
 *
 * Description:
 *   Multi-port variant of ptpd_start(). Bootstraps the daemon if it
 *   isn't running yet (only an eth_hwts port can bootstrap today —
 *   the wifi_ftm path needs an existing daemon to attach to) and
 *   configures the addressed port with the given medium. The legacy
 *   ptpd_start() is equivalent to:
 *     ptpd_start_port(0, interface, ptp_port_medium_eth_hwts)
 *
 *   The medium determines the peer-delay mechanism: eth_hwts runs the
 *   on-wire Pdelay protocol, wifi_ftm derives peer-delay from FTM
 *   (STA-side, automatic) or operates as an FTM responder (AP-side,
 *   no peer-delay measurement). The choice is taken from
 *   port[port_index].wifi_mode set via Kconfig.
 *
 * Returned Value:
 *   On success, the non-negative task ID of the PTP daemon is returned;
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_start_port(int port_index,
                    FAR const char *interface,
                    ptp_port_medium_e medium);

/****************************************************************************
 * Name: ptpd_inject_peer_delay
 *
 * Description:
 *   Push an externally-measured peer-delay sample into the running
 *   daemon for a wifi_ftm port operating as STA. The value feeds the
 *   same averaging path that the on-wire pdelay handler uses on
 *   eth_hwts ports.
 *
 ****************************************************************************/

int ptpd_inject_peer_delay(int port_index, int64_t peer_delay_ns);

/****************************************************************************
 * Name: ptpd_inject_sync
 *
 * Description:
 *   Push a marshalled FollowUpInformation TLV (per IEEE 802.1AS-2020
 *   §11.4.4 / §12.7) into the daemon for a port whose Sync transport
 *   is out-of-band (e.g. carried in an 802.11 beacon Vendor IE).
 *
 *   follow_up_info points to the FollowUpInformation byte buffer; len
 *   is the buffer length. Memory ownership stays with the caller.
 *
 ****************************************************************************/

int ptpd_inject_sync(int port_index,
                     FAR const uint8_t *follow_up_info,
                     size_t len);

/****************************************************************************
 * Name: ptpd_register_sync_egress_cb
 *
 * Description:
 *   Register a callback invoked by the daemon whenever it would emit a
 *   gPTP Sync/Follow_Up on the addressed port. The callback receives a
 *   marshalled FollowUpInformation TLV; the application then carries
 *   that payload to the wire (e.g. by updating the AP's beacon Vendor
 *   IE on a Wi-Fi port). One callback per port; passing NULL clears.
 *
 ****************************************************************************/

typedef void (*ptpd_sync_egress_cb_t)(int port_index,
                                      FAR const uint8_t *follow_up_info,
                                      size_t len,
                                      FAR void *ctx);

int ptpd_register_sync_egress_cb(int port_index,
                                 ptpd_sync_egress_cb_t cb,
                                 FAR void *ctx);

/****************************************************************************
 * Name: ptpd_now
 *
 * Description:
 *   Read the PTP-disciplined system time. This is the primary clock
 *   surface for clients that previously called
 *   clock_gettime(CLOCK_PTP_SYSTEM, ts). On platforms where
 *   CLOCK_PTP_SYSTEM is backed by the EMAC IEEE-1588 hardware clock
 *   (ESP32-P4 with esp_eth_clock), the helper returns the same value
 *   that clockid would. On platforms without 1588 hardware (e.g.
 *   ESP32-C6) the helper sources time from a software-disciplined
 *   clock backed by esp_timer_get_time() — see ptp_clock_sw_init().
 *
 *   esp_ptp must stand alone — the helper is in the ptpd_* namespace,
 *   not avb_*. (esp_avb is the primary consumer; that context is
 *   captured here in the comment only.)
 *
 * Returned Value:
 *   On success returns 0 and writes the current PTP time into *ts.
 *   On failure returns -1 with errno set, matching clock_gettime().
 *
 ****************************************************************************/

int ptpd_now(FAR struct timespec *ts);

/****************************************************************************
 * Name: ptpd_set_profile
 *
 * Description:
 *   Change the active PTP profile at runtime.
 *
 * Input Parameters:
 *   pid      - Process ID previously returned by ptpd_start()
 *   profile  - New PTP profile.
 *
 * Returned Value:
 *   On success, returns OK.
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_set_profile(int pid, ptp_profile_e profile);

/****************************************************************************
 * Name: ptpd_status
 *
 * Description:
 *   Query status from a running PTP daemon.
 *
 * Input Parameters:
 *   pid     - Process ID previously returned by ptpd_start()
 *   status  - Pointer to storage for status information.
 *
 * Returned Value:
 *   On success, returns OK.
 *   On failure, a negated errno value is returned.
 *
 * Assumptions/Limitations:
 *   Multiple threads with priority less than CONFIG_NETUTILS_PTPD_SERVERPRIO
 *   can request status simultaneously. If higher priority threads request
 *   status simultaneously, some of the requests may timeout.
 *
 ****************************************************************************/

int ptpd_status(int pid, FAR struct ptpd_status_s *status);

/****************************************************************************
 * Name: ptpd_stop
 *
 * Description:
 *   Stop PTP daemon
 *
 * Input Parameters:
 *   pid     - Process ID previously returned by ptpd_start()
 *
 * Returned Value:
 *   On success, returns OK.
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ptpd_stop(int pid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_PTPD_H */
