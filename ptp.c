/*
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2024-2026 Espressif Systems (Shanghai) CO LTD
 */

/****************************************************************************
 * apps/netutils/ptpd/ptpd.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <sys/socket.h>
#include <sys/time.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include "ptp.h"
#include "ptp_path_trace.h"
#include "ptp_message_bounds.h"
#include "ptp_signaling.h"
#include "ptp_gptp_wire.h"
#include "ptp_capable_receive.h"
#include "ptp_wired_capable.h"
#include "ptp_wifi_neighbor.h"
#include "ptp_wifi_peers.h"
#include "ptp_peer_exchange.h"
#include "ptp_peer_rate.h"
#include "ptp_peer_capability.h"

/* True once ptp_clock_sw_init() succeeds; routes time ops through the
 * SW clock backend. On C6 this is the only path that can actually move
 * the local clock rate (no working clock_adjtime(ADJ_FREQUENCY)). */
static bool s_use_sw_clock = false;

#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ptp.h"
#include "ptp_timing_snapshot.h"
#include "ptp_sync_receipt.h"
#include "ptp_wired_pi.h"
#include "freertos/FreeRTOS.h"
#include "esp_vfs_l2tap.h"
#include "esp_wifi.h"
#include "lwip/prot/ethernet.h" // Ethernet headers
#include "semaphore.h"
#include <math.h>

#include "esp_timer.h"
#include "soc/soc_caps.h"
#include <sys/timex.h>

static ptp_timing_snapshot_t s_timing_snapshot;
static ptp_timing_snapshot_t s_local_source_snapshot;
static portMUX_TYPE s_timing_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_peer_lock = portMUX_INITIALIZER_UNLOCKED;

#ifdef CONFIG_ESP_PTP_SOURCE_LOSS_PROBE
#include "ptp_source_loss_probe.h"
static ptp_source_loss_probe_t s_source_loss_probe;
#endif

static void ptp_invalidate_timing(void)
{
  portENTER_CRITICAL(&s_timing_lock);
  ++s_timing_snapshot.generation;
  s_timing_snapshot.valid = false;
  s_local_source_snapshot.valid = false;
  portEXIT_CRITICAL(&s_timing_lock);
}

uint32_t ptpd_timing_generation(void)
{
  portENTER_CRITICAL(&s_timing_lock);
  uint32_t generation = s_timing_snapshot.generation;
  portEXIT_CRITICAL(&s_timing_lock);
  return generation;
}

bool ptpd_timing_snapshot(ptp_timing_snapshot_t *snapshot)
{
  if (!snapshot) return false;
  portENTER_CRITICAL(&s_timing_lock);
  *snapshot = s_timing_snapshot;
  portEXIT_CRITICAL(&s_timing_lock);
  int64_t now_us = esp_timer_get_time();
  if (now_us < snapshot->received_us || now_us - snapshot->received_us > 500000)
    snapshot->valid = false;
  return snapshot->valid;
}

bool ptpd_time_source_snapshot(ptp_timing_snapshot_t *snapshot)
{
  if (!snapshot) return false;
  portENTER_CRITICAL(&s_timing_lock);
  *snapshot = s_local_source_snapshot.valid ? s_local_source_snapshot : s_timing_snapshot;
  portEXIT_CRITICAL(&s_timing_lock);
  int64_t now_us = esp_timer_get_time();
  if (now_us < snapshot->received_us || now_us - snapshot->received_us > 500000)
    snapshot->valid = false;
  return snapshot->valid;
}

/* Hardware PTP clock backend (esp_eth_clock + CLOCK_PTP_SYSTEM) is
 * only present on chips with an on-chip MAC (e.g. ESP32-P4). On
 * Wi-Fi-only targets like ESP32-C6 the esp_eth_clock.h header exists
 * but its CLOCK_PTP_SYSTEM / esp_eth_clock_init definitions are
 * compiled out (gated on SOC_EMAC_SUPPORTED inside the header). Fall
 * back to CLOCK_REALTIME so this file still compiles. Runtime gPTP
 * behaviour there relies on the software-clock backend in
 * ptp_clock_sw.c via ptpd_set_sw_clock_now() + ptpd_now(). */
#if SOC_EMAC_SUPPORTED
#include "esp_eth_clock.h"
#include "esp_eth_mac_esp.h"
#define PTPD_HAVE_ESP_ETH_CLOCK 1
#define PTPD_CLOCK_ID CLOCK_PTP_SYSTEM
#else
#define PTPD_HAVE_ESP_ETH_CLOCK 0
#define PTPD_CLOCK_ID CLOCK_REALTIME
#endif

#define ETH_TYPE_PTP 0x88F7

/* Lateness diagnostic: 3+ missed pdelay exchanges in a row makes
 * strict-1AS upstream bridges drop asCapable. */
/* Pdelay_Req emission is driven from esp_timer (priority 22, IRAM)
 * rather than the main poll() loop to keep cadence tight enough for
 * strict-1AS evaluators. */
static esp_timer_handle_t s_pdelay_req_timer = NULL;
static void pdelay_req_timer_cb(void *arg);

static int64_t s_ptpd_last_loop_us = 0;
static int64_t s_ptpd_loop_max_gap_us = 0;
static int64_t s_ptpd_last_report_us = 0;
static uint32_t s_ptpd_loop_iters = 0;
static uint32_t s_ptpd_tx_req_total = 0;
static uint32_t s_ptpd_tx_req_late = 0;
static int64_t s_ptpd_tx_req_late_max_us = 0;

/* Per-message-type RX counters, reset each report window. */
static uint32_t s_ptpd_rx_sync = 0;
static uint32_t s_ptpd_rx_followup = 0;
static uint32_t s_ptpd_rx_announce = 0;
static uint32_t s_ptpd_rx_pdelay_req = 0;
static uint32_t s_ptpd_rx_pdelay_resp = 0;
static uint32_t s_ptpd_rx_pdelay_fup = 0;
/* Max gap between two consecutive received syncs. If the wire shows
 * steady 125 ms syncs but this goes above say 1 s, ptpd isn't seeing
 * them even though they're reaching the NIC. */
static int64_t s_ptpd_rx_sync_last_us = 0;
static int64_t s_ptpd_rx_sync_max_gap_us = 0;

/* Hexdump of validation-rejected frames, rate-limited: the first 5,
 * then one in every 256. Enough to characterize a corrupted-delivery
 * incident without flooding the console at frame rate. */
static void ptpd_reject_dump(const char *why, const uint8_t *buf,
                             ssize_t len) {
  static uint32_t s_reject_count = 0;
  s_reject_count++;
  if (s_reject_count > 5 && (s_reject_count & 0xFF) != 0)
    return;
  char hex[3 * 24 + 1];
  int n = len < 24 ? (int)len : 24;
  for (int i = 0; i < n; i++)
    snprintf(hex + 3 * i, 4, "%02x ", buf[i]);
  ESP_LOGW("ptpd", "rx reject #%u (%s, len=%d): %s", (unsigned)s_reject_count,
           why, (int)len, hex);
}

/* L2TAP-wedge diagnosis counters: poll wakeups with revents set, read()
 * outcomes, and the last read errno. Printed in the ptpd-late line. */
static uint32_t s_ptpd_poll_wake = 0;
static uint32_t s_ptpd_read_ok = 0;
static uint32_t s_ptpd_read_err = 0;
static int s_ptpd_read_last_errno = 0;
static int s_ptpd_read_last_ret = 0;

static void ptpd_lateness_record_rx_sync(void) {
  int64_t now = esp_timer_get_time();
  if (s_ptpd_rx_sync_last_us) {
    int64_t gap = now - s_ptpd_rx_sync_last_us;
    if (gap > s_ptpd_rx_sync_max_gap_us)
      s_ptpd_rx_sync_max_gap_us = gap;
  }
  s_ptpd_rx_sync_last_us = now;
  s_ptpd_rx_sync++;
}

static void ptpd_lateness_record_tx(int64_t lateness_us) {
  s_ptpd_tx_req_total++;
  if (lateness_us > 200000LL) {
    s_ptpd_tx_req_late++;
    if (lateness_us > s_ptpd_tx_req_late_max_us)
      s_ptpd_tx_req_late_max_us = lateness_us;
  }
}

static void ptpd_lateness_tick(void) {
  int64_t now = esp_timer_get_time();
  s_ptpd_loop_iters++;
  if (s_ptpd_last_loop_us) {
    int64_t gap = now - s_ptpd_last_loop_us;
    if (gap > s_ptpd_loop_max_gap_us)
      s_ptpd_loop_max_gap_us = gap;
  }
  s_ptpd_last_loop_us = now;
  if (s_ptpd_last_report_us == 0) {
    s_ptpd_last_report_us = now;
    return;
  }
  if (now - s_ptpd_last_report_us < 10000000LL)
    return;
  ESP_LOGD(
      "ptpd-late",
      "iters=%u loop_gap_max=%lldms tx=%u/%u late(max=%lldms) "
      "rx_sync=%u max_gap=%lldms fup=%u ann=%u pd_req=%u pd_resp=%u pd_fup=%u",
      (unsigned)s_ptpd_loop_iters, s_ptpd_loop_max_gap_us / 1000,
      (unsigned)s_ptpd_tx_req_late, (unsigned)s_ptpd_tx_req_total,
      s_ptpd_tx_req_late_max_us / 1000, (unsigned)s_ptpd_rx_sync,
      s_ptpd_rx_sync_max_gap_us / 1000, (unsigned)s_ptpd_rx_followup,
      (unsigned)s_ptpd_rx_announce, (unsigned)s_ptpd_rx_pdelay_req,
      (unsigned)s_ptpd_rx_pdelay_resp, (unsigned)s_ptpd_rx_pdelay_fup);
  ESP_LOGD("ptpd-late",
           "sock: poll_wake=%u read_ok=%u read_err=%u last_ret=%d last_errno=%d",
           (unsigned)s_ptpd_poll_wake, (unsigned)s_ptpd_read_ok,
           (unsigned)s_ptpd_read_err, s_ptpd_read_last_ret,
           s_ptpd_read_last_errno);
  s_ptpd_poll_wake = 0;
  s_ptpd_read_ok = 0;
  s_ptpd_read_err = 0;
  s_ptpd_loop_max_gap_us = 0;
  s_ptpd_loop_iters = 0;
  s_ptpd_tx_req_total = 0;
  s_ptpd_tx_req_late = 0;
  s_ptpd_tx_req_late_max_us = 0;
  s_ptpd_rx_sync = 0;
  s_ptpd_rx_sync_max_gap_us = 0;
  s_ptpd_rx_followup = 0;
  s_ptpd_rx_announce = 0;
  s_ptpd_rx_pdelay_req = 0;
  s_ptpd_rx_pdelay_resp = 0;
  s_ptpd_rx_pdelay_fup = 0;
  s_ptpd_last_report_us = now;
}

#define SET_MAC_ADDR(addr, a, b, c, d, e, f)                                   \
  do {                                                                         \
    addr[0] = a;                                                               \
    addr[1] = b;                                                               \
    addr[2] = c;                                                               \
    addr[3] = d;                                                               \
    addr[4] = e;                                                               \
    addr[5] = f;                                                               \
  } while (0)

#define ERROR ESP_FAIL
#define OK ESP_OK

#define UNUSED (void)

#define MSEC_PER_SEC 1000
#define NSEC_PER_USEC 1000
#define NSEC_PER_MSEC 1000000ll
#define NSEC_PER_SEC 1000000000ll

#define CONFIG_CLOCK_ADJTIME_PERIOD_MS (CONFIG_ETH_CLOCK_ADJTIME_PERIOD_MS)
#define CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM                                     \
  (CONFIG_ETH_CLOCK_ADJTIME_SLEWLIMIT_PPB / 1000)

// To able to set either only server or only client
#ifndef CONFIG_ESP_PTP_TIMEOUT_MS
#define CONFIG_ESP_PTP_TIMEOUT_MS 0
#endif
#ifndef CONFIG_ESP_PTP_SETTIME_THRESHOLD_MS
#define CONFIG_ESP_PTP_SETTIME_THRESHOLD_MS 0
#endif
#ifndef CONFIG_ESP_PTP_MAX_PATH_DELAY_NS
#define CONFIG_ESP_PTP_MAX_PATH_DELAY_NS 0
#endif
#ifndef CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT
#define CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT 0
#endif
/* Hidden on AVB Lite endpoints, whose profile budgets one Delay_Req per
 * follower per second (profiles/avb_lite.md §1). */
#ifndef CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS
#define CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS 1000
#endif
#ifndef CONFIG_ESP_PTP_PDELAYREQ_INTERVAL_MS
#define CONFIG_ESP_PTP_PDELAYREQ_INTERVAL_MS 1000
#endif
#ifndef CONFIG_ESP_PTP_PATH_DELAY_STABILITY_NS
#define CONFIG_ESP_PTP_PATH_DELAY_STABILITY_NS 250
#endif
#ifndef CONFIG_ESP_PTP_PEER_DELAY_STABILITY_NS
#define CONFIG_ESP_PTP_PEER_DELAY_STABILITY_NS 100
#endif
#ifndef CONFIG_ESP_PTP_MAX_PEER_DELAY_NS
#define CONFIG_ESP_PTP_MAX_PEER_DELAY_NS 100000LL // 100us
#endif

#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define ADJ_FREQ_MAX 512000
/* The beacon reference carries cross-chip timing noise. Preserve its
 * gentler controller independently of the wired hardware controller. */
#define PTP_FREQ_P_DIV_SW 2000
#define PTP_FREQ_I_DIV_SW 8000    /* gentle integrator: heavily averages the gap noise
                                   * so the recovered rate stays clean (slower acquire) */
#define PTP_FREQ_STABLE_PPB 12000 /* SW: integrator flat within 12 ppm across the
                                   * window = locked (a slow wind fails this) */
#define PTP_FREQ_STABLE_WIN 6     /* plateau window (samples) for the SW lock check */
typedef struct {
  int32_t kp;
  int32_t ki;
  int32_t drift_acc;
  int64_t wired_integral_q16;
} pi_cntrl_t;

typedef union {
  struct ptp_header_s header;
  struct ptp_announce_s announce;
  struct ptp_sync_s sync;
  struct ptp_follow_up_s follow_up;
  struct ptp_delay_req_s delay_req;
  struct ptp_delay_resp_s delay_resp;
  struct ptp_delay_resp_follow_up_s delay_resp_follow_up;
  uint8_t raw[256];
} ptp_msgbuf;

/* Carrier structure for querying PTPD status */

struct ptpd_statusreq_s {
  FAR sem_t *done;
  FAR struct ptpd_status_s *dest;
};

/* Main PTPD state storage */

#define PTP_PDELAY_RESP_MAX_TRACKED 4

struct ptp_bootstrap_args_s {
  int port_index;
  ptp_port_medium_e medium;
  char interface[16];
};

struct ptp_port_s {
  /* Configuration. */
  bool enabled;
  ptp_port_medium_e medium;
  ptp_port_host_if_e host_if; /* how this port attaches to the SoC */
  ptp_port_type_e type;       /* primary / failover / bridged */
  ptp_port_wifi_mode_e
      wifi_mode;            /* ap/sta for medium=wifi_ftm, none otherwise */
  uint32_t link_speed_mbps; /* nominal PHY-rate cap */
  char interface_name[16];
  /* Speed the PHY negotiated, 0 while unknown or down. */
  uint32_t negotiated_link_mbps;
  int64_t link_speed_checked_us;
  /* EMAC timestamps every received frame (needed for tagged PTP). */
  bool rx_timestamp_all;
  bool rx_timestamp_mode_set;

  /* Link state, queryable via ptpd_port_link_up(). Defaults true so
   * the first TX cycle isn't suppressed before the first link event. */
  bool link_up;

  /* Egress callback for out-of-band Sync transports (e.g. beacon
   * Vendor IE). NULL on on-wire 802.1AS ports. */
  ptpd_sync_egress_cb_t sync_egress_cb;
  FAR void *sync_egress_ctx;

  /* Per-port runtime state. */
  uint8_t intf_hw_addr[ETH_ADDR_LEN];
  int ptp_socket;

  /* Last received packet timestamps (CLOCK_MONOTONIC). */
  struct timespec last_received_multicast;
  /* Stamped on structurally-valid frames rejected by policy (SDOID) —
   * proves the socket delivers real PTP even when nothing is accepted
   * (AVB-Lite steady state can be all-rejects on the BTC side). The
   * EMAC-wedge signature (sized frames with garbage content) fails
   * the DOMAIN check and must NOT stamp this. */
  struct timespec last_socket_alive;
  struct timespec last_received_announce;
  struct timespec last_received_sync;
  /* Historical observations only; not yet a neighbor capability state. */
  struct timespec last_received_capable_indication;
  ptp_capable_message_t last_capable_indication;
  ptp_capable_receive_t capable_receive;
  ptp_wired_capable_t wired_capable;
  ptp_wifi_neighbor_t wifi_neighbor;
  ptp_wifi_peers_t wifi_peers;
  unsigned capable_indication_count;

  /* Last transmitted packet timestamps (CLOCK_MONOTONIC). */
  struct timespec last_transmitted_sync;
  struct timespec last_transmitted_announce;
  struct timespec last_transmitted_delayresp;
  struct timespec last_transmitted_delayreq;

  /* Endpoint Declaration TLV detection — set true when an incoming
   * Pdelay_{Req,Resp,Resp_Follow_Up} on this port carries the Endpoint
   * Declaration TLV (meaning the immediate Pdelay peer is also an
   * endpoint, no boundary-clock-aware bridge sits between us on this
   * port). Triggers fallback from gPTP to standard PTP. */
  bool peer_is_endpoint;

  /* Pdelay_Resp source clockIdentity cardinality on this port. ≥2
   * distinct responders within a window indicates a flooding (non-
   * boundary-clock) L2 substrate. Tracks sourceidentity (8 bytes). */
  uint8_t pdelay_resp_responders[PTP_PDELAY_RESP_MAX_TRACKED][8];
  unsigned int pdelay_resp_responder_count;
  bool pdelay_multi_responder;

  /* Post-fallback endpoint beacon timestamp on this port. After
   * fallback to standard PTP, periodically emit a Pdelay_Req carrying
   * the Endpoint Declaration TLV. */
  struct timespec last_endpoint_beacon;

  /* Path-delay state on this port. */
  bool can_send_delayreq;
  struct timespec delayreq_time;
  int path_delay_avgcount;
  int peer_delay_avgcount;
  long path_delay_ns;
  long peer_delay_ns;
  long delayreq_interval_ms;
  long next_delayreq_interval_ms;

  /* Latest received packet on this port and its timestamp (CLOCK_REALTIME). */
  struct timespec rxtime;
  uint8_t rx_source_mac[6];
  bool rx_source_mac_valid;
  uint8_t rx_dest_mac[6];
  bool rx_dest_mac_valid;
  uint32_t rx_association;
  ptp_msgbuf rxbuf;

  /* Buffered sync packet for two-step clock setting (server sends the
   * accurate timestamp in a separate follow-up message). */
  struct ptp_sync_s twostep_packet;
  struct timespec twostep_rxtime;
  int64_t twostep_received_us;
  ptp_sync_receipt_t sync_receipt;
  bool twostep_pending;

  /* Buffered delay_resp packet for two-step peer delay measurement. */
  ptp_peer_exchange_t peer_exchange;
  ptp_peer_rate_t peer_rate;
  ptp_peer_capability_t peer_capability;
  bool capability_reported, last_reported_capability;
};

struct ptp_state_s {
  /* Request for PTPD task to stop or report status */

  bool stop;
  struct ptpd_statusreq_s status_req;

  /* Per-port array. */

  struct ptp_port_s port[CONFIG_ESP_PTP_NUM_PORTS];

  ptp_profile_e active_ptp_profile;
  /* Preferred profile. The AVB Lite fallback degrades active_ptp_profile (gPTP
   * -> standard) but leaves this untouched, so a link-up can restore the
   * preferred profile instead of being stranded in standard PTP. */
  ptp_profile_e preferred_ptp_profile;

  int64_t remote_time_ns_prev;
  int64_t local_time_ns_prev;

  int64_t last_offset_ns;
  double correction_ns;

  pi_cntrl_t offset_pi;
  int32_t freq_trim_ppb;

  /* Our own identity as a clock source */

  struct ptp_announce_s own_identity;

  /* Sequence number counters per message type */

  uint16_t announce_seq;
  uint16_t sync_seq;
  uint16_t delay_req_seq;

  /* Previous measurement and estimated clock drift rate */

  struct timespec last_delta_timestamp;
  int64_t last_delta_ns;
  int64_t last_adjtime_ns;
  long drift_avg_total_ms;
  long drift_ppb;

  /* Identity of currently selected clock source,
   * from the latest announcement message.
   *
   * The timestamps are used for timeout when a source disappears.
   * They are from the local CLOCK_MONOTONIC.
   */

  bool selected_source_valid;            /* True if operating as client */
  struct ptp_announce_s selected_source; /* Currently selected server */
  ptp_path_trace_t selected_path;
  struct timespec last_selected_announce;
  /* Source MAC of the selected timetransmitter's Announce, the
   * destination of unicast Delay_Req. */
  uint8_t selected_source_mac[6];
  bool selected_source_mac_valid;
  /* Link speed from the selected timetransmitter's Grandmaster Link
   * TLV (0 when absent) and the delayAsymmetry derived from it. */
  uint32_t selected_gm_link_mbps;
  int64_t delay_asymmetry_ns;
  /* Unicast Delay_Req bookkeeping, IEEE 1588-2019 16.9. */
  bool unicast_delay_req_outstanding;
  uint8_t unicast_delay_req_misses;
  int64_t unicast_delay_req_retry_us;

  /* gPTP → standard PTP fallback gate. One-shot per session: set true after
   * either AVB Lite §2.2 condition fires; cleared on Ethernet link-up so the
   * check re-arms for the new link. */

  bool gptp_fallback_done;
  uint8_t avb_lite_fallback_reason; /* §2.2 condition that latched, 0 none */
  bool eth_event_handler_registered;
  bool wifi_event_handler_registered;
};

#ifdef CONFIG_ESP_PTP_SERVER
#define PTPD_POLL_INTERVAL CONFIG_ESP_PTP_SYNC_INTERVAL_MS
#else
#define PTPD_POLL_INTERVAL CONFIG_ESP_PTP_TIMEOUT_MS
#endif

/* Log macros. Per-packet and per-servo-turn telemetry uses ptpdebug
 * (ESP_LOGD): compiled out at the default CONFIG_LOG_MAXIMUM_LEVEL;
 * build with DEBUG max level to re-enable for forensics. */

static const char *TAG = "ptpd";
#define ptpinfo(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define ptpwarn(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
/* Per-sync servo telemetry (~3 lines/s at 1 Hz sync). Compiled out at
 * the default log level; raise CONFIG_LOG_MAXIMUM_LEVEL to DEBUG to
 * get it back for servo forensics. */
#define ptpdebug(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define ptperr(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

static struct ptp_state_s *s_state;

/* The RX callback publishes bounded copies; only the daemon parses them. */
#define PTP_INJECT_DEPTH 8
struct ptp_injected_frame_s {
  uint16_t length;
  uint8_t port_index;
  uint32_t link_generation;
  uint8_t source_mac[6];
  bool source_mac_valid;
  struct timespec received;
  ptp_msgbuf message;
};
static struct ptp_injected_frame_s s_injected[PTP_INJECT_DEPTH];
static portMUX_TYPE s_injected_lock = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_injected_head, s_injected_count;
static uint32_t s_injected_generation[CONFIG_ESP_PTP_NUM_PORTS];
static bool s_injected_enabled;


/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint16_t s_peer_ingress_sequence, s_peer_ingress_count;

/* Receive-boundary diagnostic, independent of daemon allocation lifetime. */
void ptpd_note_wired_peer_follow_up(const uint8_t *message, size_t length) {
  if (!message || length < sizeof(struct ptp_delay_resp_follow_up_s)) return;
  uint16_t sequence = ((uint16_t)message[30] << 8) | message[31];
  portENTER_CRITICAL(&s_peer_lock);
  if (sequence == s_peer_ingress_sequence && s_peer_ingress_count < UINT16_MAX)
    s_peer_ingress_count++;
  portEXIT_CRITICAL(&s_peer_lock);
}

uint32_t ptp_wifi_link_generation(int port_index) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) return 0;
  portENTER_CRITICAL(&s_injected_lock);
  uint32_t generation = s_injected_generation[port_index];
  portEXIT_CRITICAL(&s_injected_lock);
  return generation;
}

static void ptp_clean_after_step(FAR struct ptp_state_s *state);
static void ptp_reset_for_profile(FAR struct ptp_state_s *state);
static void ptp_reset_unicast_delay_req(FAR struct ptp_state_s *state);
static void ptp_update_delay_asymmetry(FAR struct ptp_state_s *state);
static bool ptp_wired_capability(FAR struct ptp_state_s *state);

static inline bool ptp_is_gptp(FAR const struct ptp_state_s *state) {
  return state->active_ptp_profile == ptp_profile_gptp;
}

bool ptpd_wifi_sta_timing_snapshot(int port_index, ptp_wifi_sta_timing_t *snapshot) {
  if (!snapshot || port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS)
    return false;
  bool copied = false;
  portENTER_CRITICAL(&s_peer_lock);
  if (s_state) {
    const struct ptp_port_s *port = &s_state->port[port_index];
    const ptp_wifi_neighbor_t *neighbor = &port->wifi_neighbor;
    if (port->enabled && port->link_up && ptp_is_gptp(s_state) &&
        port->medium == ptp_port_medium_wifi_ftm &&
        port->wifi_mode == ptp_port_wifi_mode_sta && neighbor->associated && neighbor->bound) {
      ptp_wifi_sta_timing_t next = {.association = neighbor->association,
          .revision = neighbor->sync_revision, .stopped = neighbor->sync_stopped};
      memcpy(next.bssid, neighbor->bssid, 6);
      *snapshot = next;
      copied = true;
    }
  }
  portEXIT_CRITICAL(&s_peer_lock);
  return copied;
}

bool ptpd_wifi_association_snapshot(int port_index,
                                    ptp_wifi_association_snapshot_t *snapshot) {
  if (!snapshot || port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS)
    return false;
  bool copied = false;
  portENTER_CRITICAL(&s_peer_lock);
  if (s_state) {
    const struct ptp_port_s *port = &s_state->port[port_index];
    if (port->enabled && port->link_up && ptp_is_gptp(s_state) &&
        port->medium == ptp_port_medium_wifi_ftm &&
        port->wifi_mode == ptp_port_wifi_mode_ap &&
        ptp_wifi_peers_publishable(&port->wifi_peers))
      copied = ptp_wifi_peers_snapshot(&port->wifi_peers, port_index + 1,
                                       CONFIG_ESP_PTP_NUM_PORTS, snapshot);
  }
  portEXIT_CRITICAL(&s_peer_lock);
  return copied;
}

bool ptpd_wifi_association_reconcile(int port_index, uint32_t radio_boot,
    uint32_t radio_generation, const uint8_t macs[][6], unsigned count) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) return false;
  bool accepted = false, changed = false;
  portENTER_CRITICAL(&s_peer_lock);
  if (s_state) {
    struct ptp_port_s *port = &s_state->port[port_index];
    if (port->enabled && port->link_up && ptp_is_gptp(s_state) &&
        port->medium == ptp_port_medium_wifi_ftm && port->wifi_mode == ptp_port_wifi_mode_ap) {
      uint32_t previous = port->wifi_peers.generation;
      accepted = ptp_wifi_peers_reconcile(&port->wifi_peers, radio_boot,
                                           radio_generation, macs, count);
      changed = accepted && previous != port->wifi_peers.generation;
    }
  }
  portEXIT_CRITICAL(&s_peer_lock);
  if (changed) {
    portENTER_CRITICAL(&s_injected_lock);
    ++s_injected_generation[port_index];
    portEXIT_CRITICAL(&s_injected_lock);
  }
  return accepted;
}

uint32_t ptpd_wifi_announce_begin(int port_index, const uint8_t mac[6],
    uint32_t association, uint32_t revision, uint32_t information_revision,
    int8_t initial, int64_t now_us,
    int8_t *advertised, uint16_t *sequence) {
  if (!mac || !advertised || !sequence || port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) return 0;
  uint32_t token = 0;
  portENTER_CRITICAL(&s_peer_lock);
  if (s_state) {
    struct ptp_port_s *port = &s_state->port[port_index];
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(&port->wifi_peers, mac);
    if (port->enabled && port->link_up && ptp_is_gptp(s_state) &&
        port->medium == ptp_port_medium_wifi_ftm && port->wifi_mode == ptp_port_wifi_mode_ap &&
        ptp_wifi_peers_publishable(&port->wifi_peers) &&
        peer && peer->association == association)
      token = ptp_announce_begin(&peer->announce, initial, revision, information_revision, now_us, advertised, sequence);
  }
  portEXIT_CRITICAL(&s_peer_lock);
  return token;
}

bool ptpd_wifi_announce_finish(int port_index, const uint8_t mac[6],
    uint32_t association, uint32_t token, bool accepted, int64_t now_us) {
  if (!mac || port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) return false;
  bool current = false;
  portENTER_CRITICAL(&s_peer_lock);
  if (s_state) {
    struct ptp_port_s *port = &s_state->port[port_index];
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(&port->wifi_peers, mac);
    if (port->enabled && port->link_up && ptp_is_gptp(s_state) &&
        port->medium == ptp_port_medium_wifi_ftm && port->wifi_mode == ptp_port_wifi_mode_ap &&
        ptp_wifi_peers_publishable(&port->wifi_peers) &&
        peer && peer->association == association)
      current = ptp_announce_finish(&peer->announce, token, accepted, now_us);
  }
  portEXIT_CRITICAL(&s_peer_lock);
  return current;
}

static void ptp_arm_profile_fallback(FAR struct ptp_state_s *state) {
  /* A link change begins a fresh startup window. Restore the configured profile
   * so an AVB Lite fallback forced on a previous link (peer absent at boot, or a
   * non-AVB switch flooding Pdelay) is not permanent: once the endpoint is moved
   * onto an AVB link, link-up returns it to gPTP and re-runs the fallback
   * evaluation. The fallback degrades only the effective profile; the configured
   * profile is the intent to return to. */

  if (state->active_ptp_profile != state->preferred_ptp_profile) {
    state->active_ptp_profile = state->preferred_ptp_profile;
    ptp_reset_for_profile(state);
  }

  if (!ptp_is_gptp(state)) {
    state->avb_lite_fallback_reason = 4; /* standard PTP by configuration */
    return; /* configured as standard PTP: no gPTP fallback to arm */
  }

  state->avb_lite_fallback_reason = 0;
  state->gptp_fallback_done = false;
  state->port[0].last_transmitted_delayreq.tv_sec = 0;
  state->port[0].last_transmitted_delayreq.tv_nsec = 0;

  /* AVB Lite fallback re-evaluation on link-up (profiles/avb_lite.md §2.2). */

  state->port[0].peer_is_endpoint = false;
  portENTER_CRITICAL(&s_peer_lock);
  state->port[0].peer_exchange.lost_responses = 0;
  portEXIT_CRITICAL(&s_peer_lock);
  state->port[0].pdelay_resp_responder_count = 0;
  state->port[0].pdelay_multi_responder = false;
  memset(state->port[0].pdelay_resp_responders, 0,
         sizeof(state->port[0].pdelay_resp_responders));
  state->port[0].last_endpoint_beacon.tv_sec = 0;
  state->port[0].last_endpoint_beacon.tv_nsec = 0;

  ptpinfo("Armed profile fallback evaluation\n");
}

/* Find the first port whose medium matches; returns -1 if none. The
 * event-handler use case currently assumes at most one wired and one
 * Wi-Fi port on a given device — true for both bridge and endpoint
 * builds today. Multi-instance ETH or Wi-Fi would need the event
 * data's handle/interface to disambiguate. */
static int ptp_find_port_by_medium(FAR struct ptp_state_s *state,
                                   ptp_port_medium_e medium) {
  for (int i = 0; i < CONFIG_ESP_PTP_NUM_PORTS; ++i) {
    if (state->port[i].enabled && state->port[i].medium == medium) {
      return i;
    }
  }
  return -1;
}

static void ptp_set_port_link(FAR struct ptp_state_s *state, int port_index,
                              bool up) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return;
  }
  struct ptp_port_s *p = &state->port[port_index];
  if (p->link_up == up) {
    return;
  }
  portENTER_CRITICAL(&s_peer_lock);
  ptp_peer_invalidate(&p->peer_exchange);
  p->capable_receive.valid = false;
  portEXIT_CRITICAL(&s_peer_lock);
  portENTER_CRITICAL(&s_injected_lock);
  ++s_injected_generation[port_index];
  portEXIT_CRITICAL(&s_injected_lock);
  p->link_up = up;
  ptpinfo("port %d link %s\n", port_index, up ? "up" : "down");
}

static void ptp_eth_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
  UNUSED(event_base);
  UNUSED(event_data);
  FAR struct ptp_state_s *state = (FAR struct ptp_state_s *)arg;

  if (!state) {
    return;
  }

  int port = ptp_find_port_by_medium(state, ptp_port_medium_eth_hwts);

  if (event_id == ETHERNET_EVENT_CONNECTED) {
    ptp_set_port_link(state, port, true);
    /* A link-up event starts a new "startup" window. Resume in the current
     * profile; if that profile is gPTP, require one PDelay_Resp again. */
    ptp_arm_profile_fallback(state);
  } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
    ptp_set_port_link(state, port, false);
  }
}

static void ptp_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data) {
  UNUSED(event_base);
  UNUSED(event_data);
  FAR struct ptp_state_s *state = (FAR struct ptp_state_s *)arg;
  if (!state) {
    return;
  }
  /* STA association drives link_up on STA ports; AP is treated as
   * always-up once AP_START fires. */
  int port = ptp_find_port_by_medium(state, ptp_port_medium_wifi_ftm);
  if (port < 0) {
    return;
  }
  struct ptp_port_s *p = &state->port[port];
  switch (event_id) {
  case WIFI_EVENT_STA_CONNECTED:
    if (p->wifi_mode == ptp_port_wifi_mode_sta) {
      const wifi_event_sta_connected_t *connected = event_data;
      portENTER_CRITICAL(&s_peer_lock);
      ptp_wifi_neighbor_associate(&p->wifi_neighbor, connected ? connected->bssid : NULL);
      p->capable_receive.valid = false;
      portEXIT_CRITICAL(&s_peer_lock);
      portENTER_CRITICAL(&s_injected_lock);
      ++s_injected_generation[port];
      portEXIT_CRITICAL(&s_injected_lock);
      ptp_set_port_link(state, port, true);
    }
    break;
  case WIFI_EVENT_STA_STOP:
  case WIFI_EVENT_STA_DISCONNECTED:
    if (p->wifi_mode == ptp_port_wifi_mode_sta) {
      portENTER_CRITICAL(&s_peer_lock);
      ptp_wifi_neighbor_associate(&p->wifi_neighbor, NULL);
      p->capable_receive.valid = false;
      portEXIT_CRITICAL(&s_peer_lock);
      ptp_set_port_link(state, port, false);
    }
    break;
  case WIFI_EVENT_AP_STACONNECTED:
  case WIFI_EVENT_AP_STADISCONNECTED:
    if (p->wifi_mode == ptp_port_wifi_mode_ap && event_data) {
      portENTER_CRITICAL(&s_peer_lock);
      if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *connected = event_data;
        ptp_wifi_peers_join(&p->wifi_peers, connected->mac);
      } else {
        const wifi_event_ap_stadisconnected_t *disconnected = event_data;
        ptp_wifi_peers_leave(&p->wifi_peers, disconnected->mac);
      }
      portEXIT_CRITICAL(&s_peer_lock);
      /* Conservatively retire queued frames on every association change. */
      portENTER_CRITICAL(&s_injected_lock);
      ++s_injected_generation[port];
      portEXIT_CRITICAL(&s_injected_lock);
    }
    break;
  case WIFI_EVENT_AP_START:
    if (p->wifi_mode == ptp_port_wifi_mode_ap) {
      ptp_set_port_link(state, port, true);
    }
    break;
  case WIFI_EVENT_AP_STOP:
    if (p->wifi_mode == ptp_port_wifi_mode_ap) {
      portENTER_CRITICAL(&s_peer_lock);
      ptp_wifi_peers_clear(&p->wifi_peers);
      portEXIT_CRITICAL(&s_peer_lock);
      ptp_set_port_link(state, port, false);
    }
    break;
  default:
    break;
  }
}

static void ptp_reset_for_profile(FAR struct ptp_state_s *state) {
  /* Retire radio work before resetting interval state for the new profile. */
  portENTER_CRITICAL(&s_injected_lock);
  for (int index = 0; index < CONFIG_ESP_PTP_NUM_PORTS; ++index)
    ++s_injected_generation[index];
  portEXIT_CRITICAL(&s_injected_lock);
  portENTER_CRITICAL(&s_peer_lock);
  for (int index = 0; index < CONFIG_ESP_PTP_NUM_PORTS; ++index) {
    state->port[index].capable_receive.valid = false;
    uint32_t wired_serial = state->port[index].wired_capable.transmit.serial;
    memset(&state->port[index].wired_capable, 0, sizeof(state->port[index].wired_capable));
    state->port[index].wired_capable.transmit.serial = wired_serial;
    uint32_t sta_serial = state->port[index].wifi_neighbor.transmit.serial;
    memset(&state->port[index].wifi_neighbor.transmit, 0,
           sizeof(state->port[index].wifi_neighbor.transmit));
    state->port[index].wifi_neighbor.transmit.serial = sta_serial;
    ptp_sync_interval_reset(&state->port[index].wifi_neighbor.sync_stopped,
                             &state->port[index].wifi_neighbor.sync_revision);
    for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
      ptp_wifi_peer_t *peer = &state->port[index].wifi_peers.entries[slot];
      peer->capable.valid = false;
      ptp_sync_interval_reset(&peer->sync_stopped, &peer->sync_revision);
      ptp_announce_reset(&peer->announce);
      ptp_capable_schedule_t *schedule = &peer->transmit;
      uint32_t serial = schedule->serial;
      memset(schedule, 0, sizeof(*schedule));
      schedule->serial = serial;
    }
  }
  portEXIT_CRITICAL(&s_peer_lock);
  ptp_invalidate_timing();
  state->selected_source_valid = false;
  state->port[0].twostep_pending = false;
  memset(&state->selected_source, 0, sizeof(state->selected_source));
  memset(&state->selected_path, 0, sizeof(state->selected_path));
  memset(&state->last_selected_announce, 0, sizeof(state->last_selected_announce));
  state->selected_source_mac_valid = false;
  state->selected_gm_link_mbps = 0;
  ptp_reset_unicast_delay_req(state);
  ptp_update_delay_asymmetry(state);
  state->port[0].path_delay_avgcount = 0;
  state->port[0].path_delay_ns = 0;
  state->port[0].peer_delay_avgcount = 0;
  state->port[0].peer_delay_ns = 0;
  state->correction_ns = 0;
  state->port[0].can_send_delayreq = false;
  state->port[0].delayreq_time.tv_sec = 0;
  state->port[0].delayreq_time.tv_nsec = 0;
  state->port[0].last_transmitted_delayreq.tv_sec = 0;
  state->port[0].last_transmitted_delayreq.tv_nsec = 0;
  ptp_clean_after_step(state);

  if (ptp_is_gptp(state)) {
    state->port[0].delayreq_interval_ms =
        CONFIG_ESP_PTP_PDELAYREQ_INTERVAL_MS;
    state->port[0].next_delayreq_interval_ms =
        CONFIG_ESP_PTP_PDELAYREQ_INTERVAL_MS;
  } else {
    state->port[0].delayreq_interval_ms =
        CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS;
    state->port[0].next_delayreq_interval_ms =
        CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS;
  }
}

// Convert 8 bytes to 64-bit signed integer (nanoseconds << 16)
static int64_t get_correction_ns(uint8_t *correction_field) {
  uint64_t unsigned_correction = 0;

  // Interpret bytes as big-endian and build the value iteratively
  for (int i = 0; i < 8; i++) {
    unsigned_correction |= (uint64_t)correction_field[i] << (56 - i * 8);
  }

  // Convert to signed integer (two's complement)
  int64_t correction = (int64_t)unsigned_correction;

  // Convert from 2^16 scale to nanoseconds
  return correction >> 16;
}

// Convert period in msec to log period
static int8_t msec_to_log_period(uint16_t msec_period) {
  if (msec_period == 0)
    return 127;
  // logMessagePeriod = log2(interval_seconds)
  // Clamp between -128 and 127 as per IEEE 1588
  double log2_value = log2((double)msec_period / 1e3);
  // Round to nearest integer
  double log_period = (int8_t)round(log2_value);
  // Clamp to valid range
  if (log_period < -128.0)
    return -128;
  if (log_period > 127.0)
    return 127;
  return log_period;
}

// Convert log period to period in msec
static uint32_t log_period_to_msec(int8_t log_period) {
  // interval = 2^logMessagePeriod
  return (uint32_t)(pow(2.0, log_period) * 1e3);
}

/* Calculates randomized delay request interval in ms.
 * Range: 0.8x to 1.25x (for a given mean)
 */
uint32_t rand_delayreq_interval(uint32_t mean_interval_ms) {
  // Get raw PRNG value (0 to 2,147,483,647)
  long raw = random();

  // Normalize to 0.0 - 1.0
  double normalized = (double)raw / (double)2147483647L;

  // Scale to the gPTP jitter range (0.8 to 1.25)
  // Range width is 0.45 (1.25 - 0.80)
  double jitter_multiplier = 0.8 + (normalized * 0.45);

  return (uint32_t)(mean_interval_ms * jitter_multiplier);
}

static int ptp_get_esp_eth_handle(struct ptp_state_s *state,
                                  esp_eth_handle_t *eth_handle) {
  return ioctl(state->port[0].ptp_socket, L2TAP_G_DEVICE_DRV_HNDL, eth_handle);
}

static void ptp_create_eth_frame_to(struct ptp_state_s *state,
                                    uint8_t *eth_frame, void *ptp_msg,
                                    uint16_t ptp_msg_len,
                                    const uint8_t *dest_mac) {
  struct eth_hdr eth_hdr = {.type = htons(ETH_TYPE_PTP)};

  memcpy(&eth_hdr.dest.addr, dest_mac, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->port[0].intf_hw_addr, ETH_ADDR_LEN);

  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  memcpy(eth_frame + sizeof(eth_hdr), ptp_msg, ptp_msg_len);
  ptp_gptp_wire_normalize(eth_frame + sizeof(eth_hdr), ptp_msg_len, ptp_is_gptp(state));
}

static void ptp_create_eth_frame(struct ptp_state_s *state, uint8_t *eth_frame,
                                 void *ptp_msg, uint16_t ptp_msg_len) {
  ptp_create_eth_frame_to(state, eth_frame, ptp_msg, ptp_msg_len,
                          ptp_is_gptp(state) ? LLDP_MULTICAST_ADDR
                                             : PTP4L_MULTICAST_ADDR);
}

#define PTP_VLAN_TAG_LEN 4
#define PTP_PRIORITY_TAG_PCP 7

/* AVB Lite PTP goes priority-tagged, VLAN 0 PCP 7 (profiles/avb_lite.md
 * §5). gPTP and the bridge-group beacon stay untagged. */
static bool ptp_tags_frame(FAR const struct ptp_state_s *state,
                           const uint8_t *dest_mac) {
#ifdef CONFIG_ESP_PTP_LITE_PRIORITY_TAG
  return !ptp_is_gptp(state) &&
         memcmp(dest_mac, LLDP_MULTICAST_ADDR, ETH_ADDR_LEN) != 0;
#else
  (void)state;
  (void)dest_mac;
  return false;
#endif
}

/* L2TAP writes only frames of its own ethertype, so tagged frames go
 * straight to the driver, which still returns the descriptor TX
 * timestamp. */
static int ptp_net_send_tagged(FAR struct ptp_state_s *state, void *ptp_msg,
                               uint16_t ptp_msg_len, struct timespec *ts,
                               const uint8_t *dest_mac) {
  esp_eth_handle_t eth_handle;
  if (ptp_get_esp_eth_handle(state, &eth_handle) < 0) {
    return ERROR;
  }
  uint8_t eth_frame[ETH_HEADER_LEN + PTP_VLAN_TAG_LEN + ptp_msg_len];
  memcpy(eth_frame, dest_mac, ETH_ADDR_LEN);
  memcpy(eth_frame + ETH_ADDR_LEN, state->port[0].intf_hw_addr, ETH_ADDR_LEN);
  eth_frame[12] = 0x81;
  eth_frame[13] = 0x00;
  eth_frame[14] = PTP_PRIORITY_TAG_PCP << 5; /* VLAN ID 0 */
  eth_frame[15] = 0x00;
  eth_frame[16] = ETH_TYPE_PTP >> 8;
  eth_frame[17] = ETH_TYPE_PTP & 0xFF;
  uint8_t *payload = eth_frame + ETH_HEADER_LEN + PTP_VLAN_TAG_LEN;
  memcpy(payload, ptp_msg, ptp_msg_len);
  ptp_gptp_wire_normalize(payload, ptp_msg_len, ptp_is_gptp(state));

  eth_mac_time_t tx_time = {0};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  esp_err_t err = esp_eth_transmit_ctrl_vargs(eth_handle, ts ? &tx_time : NULL,
                                              2, eth_frame,
                                              (uint32_t)sizeof(eth_frame));
#pragma GCC diagnostic pop
  if (err != ESP_OK) {
    errno = (err == ESP_ERR_TIMEOUT) ? EAGAIN : EIO;
    return ERROR;
  }
  if (ts) {
    ts->tv_sec = tx_time.seconds;
    ts->tv_nsec = tx_time.nanoseconds;
  }
  return (int)sizeof(eth_frame);
}

static int ptp_net_send_to(FAR struct ptp_state_s *state, void *ptp_msg,
                           uint16_t ptp_msg_len, struct timespec *ts,
                           const uint8_t *dest_mac) {
  if (ptp_tags_frame(state, dest_mac)) {
    return ptp_net_send_tagged(state, ptp_msg, ptp_msg_len, ts, dest_mac);
  }
  uint8_t eth_frame[ptp_msg_len + ETH_HEADER_LEN];
  ptp_create_eth_frame_to(state, eth_frame, ptp_msg, ptp_msg_len, dest_mac);

  // wrap "Info Records Buffer" into union to ensure proper alignment of data
  // (this is typically needed when accessing double word variables or structs
  // containing double word variables)
  union {
    uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
    l2tap_irec_hdr_t align;
  } u;

  l2tap_extended_buff_t ptp_msg_ext_buff;

  ptp_msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  ptp_msg_ext_buff.info_recs_buff = u.info_recs_buff;
  ptp_msg_ext_buff.buff = eth_frame;
  ptp_msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ptp_msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  int ret = write(state->port[0].ptp_socket, &ptp_msg_ext_buff, 0);

  // check if write was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;
  }

  return ret;
}

static int ptp_net_send(FAR struct ptp_state_s *state, void *ptp_msg,
                        uint16_t ptp_msg_len, struct timespec *ts) {
  return ptp_net_send_to(state, ptp_msg, ptp_msg_len, ts,
                         ptp_is_gptp(state) ? LLDP_MULTICAST_ADDR
                                            : PTP4L_MULTICAST_ADDR);
}

static int ptp_net_recv(FAR struct ptp_state_s *state, void *ptp_msg,
                        uint16_t ptp_msg_len, struct timespec *ts) {
  uint8_t eth_frame[ptp_msg_len + ETH_HEADER_LEN + PTP_VLAN_TAG_LEN];

  // wrap "Info Records Buffer" into union to ensure proper alignment of data
  // (this is typically needed when accessing double word variables or structs
  // containing double word variables)
  union {
    uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
    l2tap_irec_hdr_t align;
  } u;
  l2tap_extended_buff_t ptp_msg_ext_buff;

  ptp_msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  ptp_msg_ext_buff.info_recs_buff = u.info_recs_buff;
  ptp_msg_ext_buff.buff = eth_frame;
  ptp_msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&ptp_msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  int ret = read(state->port[0].ptp_socket, &ptp_msg_ext_buff, 0);
  s_ptpd_read_last_ret = ret;
  if (ret > 0) {
    s_ptpd_read_ok++;
  } else {
    s_ptpd_read_err++;
    s_ptpd_read_last_errno = errno;
  }

  // check if read was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;
  }

  if (ret <= ETH_HEADER_LEN) {
    return ERROR;
  }

  /* Accept PTP priority-tagged as well as untagged (avb_lite.md §5). */
  size_t header_len = ETH_HEADER_LEN;
  if (ret > ETH_HEADER_LEN + PTP_VLAN_TAG_LEN && eth_frame[12] == 0x81 &&
      eth_frame[13] == 0x00 && eth_frame[16] == (ETH_TYPE_PTP >> 8) &&
      eth_frame[17] == (ETH_TYPE_PTP & 0xFF)) {
    header_len += PTP_VLAN_TAG_LEN;
  }

  /* Unicast addressed to another station only shows up in promiscuous
   * mode; it is not for this clock. */
  if (!(eth_frame[0] & 0x01) &&
      memcmp(eth_frame, state->port[0].intf_hw_addr, ETH_ADDR_LEN) != 0) {
    return ERROR;
  }

  size_t payload_len = (size_t)ret - header_len;
  if (payload_len > ptp_msg_len) {
    payload_len = ptp_msg_len;
  }

  memcpy(state->port[0].rx_dest_mac, eth_frame, ETH_ADDR_LEN);
  state->port[0].rx_dest_mac_valid = true;
  memcpy(state->port[0].rx_source_mac, eth_frame + ETH_ADDR_LEN, ETH_ADDR_LEN);
  state->port[0].rx_source_mac_valid = true;
  memcpy(ptp_msg, &eth_frame[header_len], payload_len);

  return (int)payload_len;
}

static int64_t timespec_to_ns(FAR const struct timespec *ts) {
  return ts->tv_sec * NSEC_PER_SEC + (ts->tv_nsec);
}

/* Convert from timespec to PTP format */

static void timespec_to_ptp_format(FAR struct timespec *ts,
                                   FAR uint8_t *timestamp) {
  /* IEEE 1588 uses 48 bits for seconds and 32 bits for nanoseconds,
   * both fields big-endian.
   */

#ifdef CONFIG_SYSTEM_TIME64
  timestamp[0] = (uint8_t)(ts->tv_sec >> 40);
  timestamp[1] = (uint8_t)(ts->tv_sec >> 32);
#else
  timestamp[0] = 0;
  timestamp[1] = 0;
#endif
  timestamp[2] = (uint8_t)(ts->tv_sec >> 24);
  timestamp[3] = (uint8_t)(ts->tv_sec >> 16);
  timestamp[4] = (uint8_t)(ts->tv_sec >> 8);
  timestamp[5] = (uint8_t)(ts->tv_sec >> 0);

  timestamp[6] = (uint8_t)(ts->tv_nsec >> 24);
  timestamp[7] = (uint8_t)(ts->tv_nsec >> 16);
  timestamp[8] = (uint8_t)(ts->tv_nsec >> 8);
  timestamp[9] = (uint8_t)(ts->tv_nsec >> 0);
}

/* Convert from PTP format to timespec */

static void ptp_format_to_timespec(FAR const uint8_t *timestamp,
                                   FAR struct timespec *ts) {
  ts->tv_sec =
      (((int64_t)timestamp[0]) << 40) | (((int64_t)timestamp[1]) << 32) |
      (((int64_t)timestamp[2]) << 24) | (((int64_t)timestamp[3]) << 16) |
      (((int64_t)timestamp[4]) << 8) | (((int64_t)timestamp[5]) << 0);

  ts->tv_nsec = (((long)timestamp[6]) << 24) | (((long)timestamp[7]) << 16) |
                (((long)timestamp[8]) << 8) | (((long)timestamp[9]) << 0);
}

/* Returns true if A is a better clock source than B.
 * Implements the BTCA from IEEE 1588-2019 §9.3 and
 * IEEE 802.1AS-2020 §10.3. */

static bool is_better_clock(FAR const struct ptp_announce_s *a,
                            FAR const struct ptp_announce_s *b) {
  /* System identity comparison per IEEE 1588 §9.3.4 / 802.1AS §10.3.5:
   * priority1, clock class, accuracy, variance, priority2, clock
   * identity — each field consulted only when all earlier ones are
   * equal. The fields sit contiguous in wire order (which IS the
   * comparison order), so a single memcmp over the 14 bytes from
   * btc_priority1 through btc_identity is the lexicographic compare.
   * The previous flat OR of per-field comparisons let a clock that
   * LOST priority1 still win on any numerically-lower later field
   * (observed: a peer with worse priority1 but lower accuracy byte
   * was selected as BTC, silencing our own announce). */
  int system_identity_check =
      memcmp(&a->btc_priority1, &b->btc_priority1,
             1 + sizeof(a->btc_quality) + 1 + sizeof(a->btc_identity));

  // Check if A is a better clock source than B
  if ((system_identity_check < 0) /* Compare root system identity */
      || ((system_identity_check == 0) &&
          (memcmp(a->stepsremoved, b->stepsremoved, sizeof(a->stepsremoved)) <
           0)) /* Compare steps removed */
      || ((system_identity_check == 0) &&
          (memcmp(a->stepsremoved, b->stepsremoved, sizeof(a->stepsremoved)) ==
           0) &&
          (memcmp(a->header.sourceidentity, b->header.sourceidentity,
                  sizeof(a->header.sourceidentity)) <
           0)) /* Compare source port identity */
      || ((system_identity_check == 0) &&
          (memcmp(a->stepsremoved, b->stepsremoved, sizeof(a->stepsremoved)) ==
           0) &&
          (memcmp(a->header.sourceidentity, b->header.sourceidentity,
                  sizeof(a->header.sourceidentity)) == 0) &&
          (memcmp(a->header.sourceportindex, b->header.sourceportindex,
                  sizeof(a->header.sourceportindex)) <
           0))) /* Compare port number */
  {
    return true;
  } else {
    return false;
  }
}

static int64_t timespec_to_ms(FAR const struct timespec *ts) {
  return ts->tv_sec * MSEC_PER_SEC + (ts->tv_nsec / NSEC_PER_MSEC);
}

/* Get positive or negative delta between two timespec values.
 * If value would exceed int64 limit (292 years), return INT64_MAX/MIN.
 */

static int64_t timespec_delta_ns(FAR const struct timespec *ts1,
                                 FAR const struct timespec *ts2) {
  int64_t delta_s;

  delta_s = ts1->tv_sec - ts2->tv_sec;

#ifdef CONFIG_SYSTEM_TIME64
  /* Conversion to nanoseconds could overflow if the system time is 64-bit */

  if (delta_s >= INT64_MAX / NSEC_PER_SEC) {
    return INT64_MAX;
  } else if (delta_s <= INT64_MIN / NSEC_PER_SEC) {
    return INT64_MIN;
  }
#endif

  return delta_s * NSEC_PER_SEC + (ts1->tv_nsec - ts2->tv_nsec);
}

static int64_t ptp_announce_receipt_timeout_ns(const struct ptp_announce_s *msg) {
  int8_t interval = (int8_t)msg->header.logmessageinterval;
  if (interval < -24 || interval > 24)
    return INT64_C(3000000) * CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS;
  int64_t timeout_ns = INT64_C(3000000000);
  if (interval >= 0)
    return timeout_ns << interval;
  int64_t divisor = INT64_C(1) << -interval;
  return (timeout_ns + divisor - 1) / divisor;
}

/* Check if the currently selected source is still valid */

static bool is_selected_source_valid(FAR struct ptp_state_s *state) {
  if (ptp_is_gptp(state) && state->port[0].medium == ptp_port_medium_eth_hwts &&
      !ptp_wired_capability(state)) return false;
  struct timespec time_now;
  struct timespec delta;

  if ((state->selected_source.header.messagetype & PTP_MSGTYPE_MASK) !=
      PTP_MSGTYPE_ANNOUNCE) {
    return false; /* Uninitialized value */
  }

  /* Announce receipt is independent of Sync and unrelated peer traffic. */
  clock_gettime(CLOCK_MONOTONIC, &time_now);
  if (ptp_is_gptp(state)) {
    int64_t announce_age = timespec_delta_ns(&time_now, &state->last_selected_announce);
    if (announce_age < 0 ||
        announce_age >= ptp_announce_receipt_timeout_ns(&state->selected_source))
      return false;
    if (state->selected_source.btc_priority1 < 255 &&
        !ptp_sync_receipt_current(&state->port[0].sync_receipt, esp_timer_get_time()))
      return false;
    return true;
  }
  struct timespec last_evt = state->port[0].last_received_sync;
  if (state->last_selected_announce.tv_sec > last_evt.tv_sec ||
      (state->last_selected_announce.tv_sec == last_evt.tv_sec &&
       state->last_selected_announce.tv_nsec > last_evt.tv_nsec)) {
    last_evt = state->last_selected_announce;
  }
  clock_timespec_subtract(&time_now, &last_evt, &delta);

  if (timespec_to_ms(&delta) > CONFIG_ESP_PTP_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Too long time since received packet\n");
    return false; /* Too long time since received packet */
  }

  return true;
}

bool ptpd_ftm_source_observed(int port_index, const uint8_t source_port[10],
                             uint8_t domain, int8_t log_interval, int64_t received_us, uint32_t *generation)
{
  if (!s_state || !source_port || !generation || !s_use_sw_clock ||
      port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS ||
      !s_state->port[port_index].link_up || !s_state->selected_source_valid ||
      !ptp_is_gptp(s_state) || s_state->selected_source.btc_priority1 == 255 ||
      domain != CONFIG_ESP_PTP_DOMAIN ||
      memcmp(source_port, s_state->selected_source.header.sourceidentity, 8) ||
      memcmp(source_port + 8, s_state->selected_source.header.sourceportindex, 2)) return false;
  if (!ptp_sync_receipt_observe(&s_state->port[port_index].sync_receipt,
      received_us, log_interval, esp_timer_get_time())) return false;
  ptp_timing_snapshot_t snapshot;
  ptpd_timing_snapshot(&snapshot);
  *generation = snapshot.generation;
  clock_gettime(CLOCK_MONOTONIC, &s_state->port[port_index].last_received_sync);
  return true;
}

extern void ptp_ftm_daemon_tick(void) __attribute__((weak));
extern bool ptp_ftm_clock_ready(uint32_t generation) __attribute__((weak));

/* Increment sequence number for packet type, and copy to header */

static void ptp_increment_sequence(FAR uint16_t *sequence_num,
                                   FAR struct ptp_header_s *hdr) {
  *sequence_num += 1;
  hdr->sequenceid[0] = (uint8_t)(*sequence_num >> 8);
  hdr->sequenceid[1] = (uint8_t)(*sequence_num);
}

/* Get sequence number from received packet */

static uint16_t ptp_get_sequence(FAR const struct ptp_header_s *hdr) {
  return ((uint16_t)hdr->sequenceid[0] << 8) | hdr->sequenceid[1];
}

/* Get current system timestamp as a timespec
 * TODO: Possibly add support for selecting different clock or using
 *       architecture-specific interface for clock access.
 */

static int ptp_gettime(FAR struct ptp_state_s *state, FAR struct timespec *ts) {
  UNUSED(state);
  if (s_use_sw_clock) {
    return ptp_clock_sw_now(ts);
  }
  return clock_gettime(PTPD_CLOCK_ID, ts);
}

/* Change current system timestamp by jumping */

static int ptp_settime(FAR struct ptp_state_s *state, FAR struct timespec *ts) {
  UNUSED(state);
  if (s_use_sw_clock) {
    return ptp_clock_sw_settime(ts);
  }
  return clock_settime(PTPD_CLOCK_ID, ts);
}

/* Smoothly adjust timestamp. */

static int ptp_adjtime(FAR struct ptp_state_s *state, int64_t delta_ns) {
  if (s_use_sw_clock) {
    return ptp_clock_sw_adjtime_offset(delta_ns);
  }
  struct timex tx = {
      .modes = ADJ_OFFSET | ADJ_NANO,
      .offset = (long)delta_ns,
  };
  return clock_adjtime(PTPD_CLOCK_ID, &tx);
}

/* Translate per-port Kconfig topology knobs into runtime ptp_port_s. */
static void ptp_apply_port_topology_kconfig(FAR struct ptp_state_s *state) {
  /* ---- Port 0 ---- */
#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_ETH_HWTS)
  state->port[0].medium = ptp_port_medium_eth_hwts;
#elif defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM)
  state->port[0].medium = ptp_port_medium_wifi_ftm;
#endif
#if defined(CONFIG_ESP_PTP_PORT0_HOST_IF_EMAC)
  state->port[0].host_if = ptp_port_host_if_emac;
#elif defined(CONFIG_ESP_PTP_PORT0_HOST_IF_AHB)
  state->port[0].host_if = ptp_port_host_if_ahb;
#elif defined(CONFIG_ESP_PTP_PORT0_HOST_IF_SDIO)
  state->port[0].host_if = ptp_port_host_if_sdio;
#elif defined(CONFIG_ESP_PTP_PORT0_HOST_IF_SPI)
  state->port[0].host_if = ptp_port_host_if_spi;
#elif defined(CONFIG_ESP_PTP_PORT0_HOST_IF_USB)
  state->port[0].host_if = ptp_port_host_if_usb;
#else
  state->port[0].host_if = ptp_port_host_if_other;
#endif
#if defined(CONFIG_ESP_PTP_PORT0_TYPE_FAILOVER)
  state->port[0].type = ptp_port_type_failover;
#elif defined(CONFIG_ESP_PTP_PORT0_TYPE_BRIDGED)
  state->port[0].type = ptp_port_type_bridged;
#else
  state->port[0].type = ptp_port_type_primary;
#endif
#if defined(CONFIG_ESP_PTP_PORT0_WIFI_MODE_AP)
  state->port[0].wifi_mode = ptp_port_wifi_mode_ap;
#elif defined(CONFIG_ESP_PTP_PORT0_WIFI_MODE_STA)
  state->port[0].wifi_mode = ptp_port_wifi_mode_sta;
#else
  state->port[0].wifi_mode = ptp_port_wifi_mode_none;
#endif
#ifdef CONFIG_ESP_PTP_PORT0_LINK_SPEED_MBPS
  state->port[0].link_speed_mbps = CONFIG_ESP_PTP_PORT0_LINK_SPEED_MBPS;
#else
  state->port[0].link_speed_mbps = 0;
#endif

  /* ---- Port 1 ---- */
#if CONFIG_ESP_PTP_NUM_PORTS > 1
#if defined(CONFIG_ESP_PTP_PORT1_MEDIUM_ETH_HWTS)
  state->port[1].medium = ptp_port_medium_eth_hwts;
#elif defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM)
  state->port[1].medium = ptp_port_medium_wifi_ftm;
#endif
#if defined(CONFIG_ESP_PTP_PORT1_HOST_IF_EMAC)
  state->port[1].host_if = ptp_port_host_if_emac;
#elif defined(CONFIG_ESP_PTP_PORT1_HOST_IF_AHB)
  state->port[1].host_if = ptp_port_host_if_ahb;
#elif defined(CONFIG_ESP_PTP_PORT1_HOST_IF_SDIO)
  state->port[1].host_if = ptp_port_host_if_sdio;
#elif defined(CONFIG_ESP_PTP_PORT1_HOST_IF_SPI)
  state->port[1].host_if = ptp_port_host_if_spi;
#elif defined(CONFIG_ESP_PTP_PORT1_HOST_IF_USB)
  state->port[1].host_if = ptp_port_host_if_usb;
#else
  state->port[1].host_if = ptp_port_host_if_other;
#endif
#if defined(CONFIG_ESP_PTP_PORT1_TYPE_FAILOVER)
  state->port[1].type = ptp_port_type_failover;
#elif defined(CONFIG_ESP_PTP_PORT1_TYPE_BRIDGED)
  state->port[1].type = ptp_port_type_bridged;
#else
  state->port[1].type = ptp_port_type_primary;
#endif
#if defined(CONFIG_ESP_PTP_PORT1_WIFI_MODE_AP)
  state->port[1].wifi_mode = ptp_port_wifi_mode_ap;
#elif defined(CONFIG_ESP_PTP_PORT1_WIFI_MODE_STA)
  state->port[1].wifi_mode = ptp_port_wifi_mode_sta;
#else
  state->port[1].wifi_mode = ptp_port_wifi_mode_none;
#endif
#ifdef CONFIG_ESP_PTP_PORT1_LINK_SPEED_MBPS
  state->port[1].link_speed_mbps = CONFIG_ESP_PTP_PORT1_LINK_SPEED_MBPS;
#else
  state->port[1].link_speed_mbps = 0;
#endif
#endif /* CONFIG_ESP_PTP_NUM_PORTS > 1 */
}

/* Recreate the L2TAP socket of a wired port in place. Recovery path
 * for a wedged fd: a multi-second stall of the daemon loop (observed
 * with bursts of NVS persist writes) can leave the L2TAP fd
 * permanently silent — the EMAC dispatcher keeps handing 0x88f7
 * frames to esp_vfs_l2tap_eth_filter_frame, but poll() on the old fd
 * never signals again. Closing and reopening the fd restores RX.
 * Only the socket-level state is rebuilt; the EMAC PTP clock and MAC
 * filters set up at init are untouched. */
static int ptp_port_reopen_l2tap(FAR struct ptp_state_s *state,
                                 int port_index) {
  struct ptp_port_s *p = &state->port[port_index];
  if (p->medium != ptp_port_medium_eth_hwts)
    return ERROR;
  if (p->ptp_socket >= 0) {
    close(p->ptp_socket);
    p->ptp_socket = -1;
  }
  int fd = open("/dev/net/tap", 0);
  if (fd < 0) {
    ptperr("port %d: L2TAP reopen failed: %d\n", port_index, errno);
    return ERROR;
  }
  uint16_t eth_type_filter = ETH_TYPE_PTP;
  if (ioctl(fd, L2TAP_S_INTF_DEVICE, p->interface_name) < 0 ||
      ioctl(fd, L2TAP_S_RCV_FILTER, &eth_type_filter) < 0 ||
      ioctl(fd, L2TAP_S_TIMESTAMP_EN) < 0) {
    ptperr("port %d: L2TAP reopen config failed: %d\n", port_index, errno);
    close(fd);
    return ERROR;
  }
  p->ptp_socket = fd;
  ptpwarn("port %d: L2TAP socket reopened after RX starvation\n", port_index);
  return OK;
}

/* Per-medium port initialisation. Caller has filled
 * enabled/medium/interface_name/wifi_mode; helper fills intf_hw_addr
 * and any medium-specific resources. */

static int ptp_port_init_eth_hwts(FAR struct ptp_state_s *state, int port_index,
                                  FAR const char *interface) {
  struct ptp_port_s *p = &state->port[port_index];

  p->ptp_socket = open("/dev/net/tap", 0);
  if (p->ptp_socket < 0) {
    ptperr("port %d: failed to create L2TAP socket: %d\n", port_index, errno);
    return ERROR;
  }
  if (ioctl(p->ptp_socket, L2TAP_S_INTF_DEVICE, interface) < 0) {
    ptperr("port %d: failed to bind L2TAP to interface \"%s\": %d\n",
           port_index, interface, errno);
    return ERROR;
  }
  uint16_t eth_type_filter = ETH_TYPE_PTP;
  if (ioctl(p->ptp_socket, L2TAP_S_RCV_FILTER, &eth_type_filter) < 0) {
    ptperr("port %d: failed to set L2TAP ethertype filter: %d\n", port_index,
           errno);
    return ERROR;
  }
  esp_eth_handle_t eth_handle;
  if (ioctl(p->ptp_socket, L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
    ptperr("port %d: failed to fetch eth_handle from L2TAP: %d\n", port_index,
           errno);
    return ERROR;
  }
#if PTPD_HAVE_ESP_ETH_CLOCK
  esp_eth_clock_cfg_t clk_cfg = {.clock_id = PTPD_CLOCK_ID};
  if (esp_eth_clock_init(eth_handle, &clk_cfg) != ESP_OK) {
    ptperr("port %d: failed to initialise EMAC PTP clock\n", port_index);
    return ERROR;
  }
#else
  /* Target without on-chip 1588 hardware (e.g. C6 wired via SDIO).
   * ptpd_now() routes through the software-clock backend instead. */
  (void)eth_handle;
#endif
  if (ioctl(p->ptp_socket, L2TAP_S_TIMESTAMP_EN) < 0) {
    ptperr("port %d: failed to enable L2TAP timestamping: %d\n", port_index,
           errno);
    return ERROR;
  }

  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &p->intf_hw_addr);

  uint8_t dest_addr[ETH_ADDR_LEN];
  SET_MAC_ADDR(dest_addr, 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00);
  esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);
  SET_MAC_ADDR(dest_addr, 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E);
  esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);

  /* Register the ETH_EVENT handler once per daemon. */
  if (!state->eth_event_handler_registered) {
    if (esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                   ptp_eth_event_handler, state) == ESP_OK) {
      state->eth_event_handler_registered = true;
    } else {
      ptpwarn("port %d: failed to register ETH_EVENT handler; gPTP "
              "link-up fallback check will only run at daemon startup\n",
              port_index);
    }
  }

  return OK;
}

static int ptp_port_init_wifi_ftm(FAR struct ptp_state_s *state, int port_index,
                                  FAR const char *interface) {
  (void)interface; /* label only on this medium — no socket to bind */
  struct ptp_port_s *p = &state->port[port_index];

  /* No L2TAP socket: Sync rides the SoftAP's Beacon Vendor IE via the
   * sync_egress_cb and peer-delay is FTM-driven via inject_peer_delay. */
  p->ptp_socket = -1;

  wifi_interface_t wifi_if;
  if (p->wifi_mode == ptp_port_wifi_mode_ap) {
    wifi_if = WIFI_IF_AP;
  } else if (p->wifi_mode == ptp_port_wifi_mode_sta) {
    wifi_if = WIFI_IF_STA;
  } else {
    ptperr("port %d (wifi_ftm): wifi_mode not set — check "
           "ESP_PTP_PORT%d_WIFI_MODE_{AP,STA} Kconfig\n",
           port_index, port_index);
    return ERROR;
  }
  if (esp_wifi_get_mac(wifi_if, p->intf_hw_addr) != ESP_OK) {
    ptperr("port %d (wifi_ftm): esp_wifi_get_mac(%s) failed — Wi-Fi "
           "must be initialised (esp_wifi_init/start) before this call\n",
           port_index, wifi_if == WIFI_IF_AP ? "WIFI_IF_AP" : "WIFI_IF_STA");
    return ERROR;
  }

  /* Drive link_up from STA_{CONNECTED,DISCONNECTED} and AP_{START,STOP}. */
  if (!state->wifi_event_handler_registered) {
    if (esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   ptp_wifi_event_handler, state) == ESP_OK) {
      state->wifi_event_handler_registered = true;
    } else {
      ptpwarn("port %d: failed to register WIFI_EVENT handler; link_up "
              "will stay at its default — consumers querying "
              "ptpd_port_link_up() may not back off TX during outages\n",
              port_index);
    }
  }

  if (p->wifi_mode == ptp_port_wifi_mode_ap) {
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
      portENTER_CRITICAL(&s_peer_lock);
      uint32_t generation = p->wifi_peers.generation;
      portEXIT_CRITICAL(&s_peer_lock);
      wifi_sta_list_t stations = {0};
      esp_err_t listed = esp_wifi_ap_get_sta_list(&stations);
      portENTER_CRITICAL(&s_peer_lock);
      bool current = p->wifi_peers.generation == generation;
      if (listed == ESP_OK && current)
        for (int index = 0; index < stations.num; ++index)
          if (!ptp_wifi_peers_find(&p->wifi_peers, stations.sta[index].mac))
            ptp_wifi_peers_join(&p->wifi_peers, stations.sta[index].mac);
      portEXIT_CRITICAL(&s_peer_lock);
      if (listed != ESP_OK || current) break;
    }
  }

  /* STA-mode wifi_ftm port: hand off the §12.7 beacon-IE parser, the
   * FTM initiator loop, and the FTM_REPORT handler to the dedicated
   * Wi-Fi STA transport module. Application code (e.g.
   * ESP-AVB-Endpoint) no longer needs to know about §12.7 IE bytes or
   * FTM cadence — bringing the port up is the only integration call. */
  if (p->wifi_mode == ptp_port_wifi_mode_sta) {
    portENTER_CRITICAL(&s_peer_lock);
    uint32_t association = p->wifi_neighbor.association;
    portEXIT_CRITICAL(&s_peer_lock);
    wifi_ap_record_t current_ap;
    esp_err_t associated = esp_wifi_sta_get_ap_info(&current_ap);
    portENTER_CRITICAL(&s_peer_lock);
    if (associated == ESP_OK && p->wifi_neighbor.association == association &&
        !p->wifi_neighbor.associated)
      ptp_wifi_neighbor_associate(&p->wifi_neighbor, current_ap.bssid);
    portEXIT_CRITICAL(&s_peer_lock);
    if (ptp_wifi_sta_start(port_index) != 0) {
      ptpwarn("port %d (wifi_ftm STA): ptp_wifi_sta_start failed — "
              "Sync via beacon-IE / FTM pair injection will not run\n",
              port_index);
    }
  }

#ifdef CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR
  /* AP-mode wifi_ftm port: register the beacon-IE FollowUp publisher
   * now that s_state is guaranteed to be non-NULL. Was previously
   * driven off WIFI_EVENT_AP_START in ptp_beacon_ie.c, which fired
   * during init_wifi_softap — before this code path's ptpd_start_port
   * call had spawned the daemon — so the registration was permanently
   * dead-on-arrival with -ESRCH for the whole boot. */
  if (p->wifi_mode == ptp_port_wifi_mode_ap) {
    if (ptp_beacon_ie_attach(port_index) != 0) {
      ptpwarn("port %d (wifi_ftm AP): ptp_beacon_ie_attach failed — "
              "STAs will not see §12.7 FollowUpInformation in beacons\n",
              port_index);
    }
  }
#endif

  return OK;
}

static int ptp_initialize_state(FAR struct ptp_state_s *state,
                                FAR const struct ptp_bootstrap_args_s *args) {
  if (args->port_index < 0 || args->port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    ptperr("bootstrap port_index %d out of range [0,%d)\n", args->port_index,
           CONFIG_ESP_PTP_NUM_PORTS);
    return ERROR;
  }

  /* Daemon-wide state: servo, profile, time baselines. */
  state->remote_time_ns_prev = 0;
  state->local_time_ns_prev = 0;
  state->offset_pi.kp = 1;
  state->offset_pi.ki = 3; /* matches ptp4l default gain ~0.3 */
  state->offset_pi.drift_acc = 0;

#ifdef CONFIG_ESP_PTP_GPTP_PROFILE
  state->active_ptp_profile = ptp_profile_gptp;
#else
  state->active_ptp_profile = ptp_profile_standard;
#endif
  state->preferred_ptp_profile = state->active_ptp_profile;
  ptp_reset_for_profile(state);

  /* Per-port topology from Kconfig; runs before per-medium init so
   * helpers can read wifi_mode etc. on the bootstrap port. */
  ptp_apply_port_topology_kconfig(state);

  /* Seed the bootstrap port's API-level config — the runtime fields
   * (intf_hw_addr, ptp_socket) are filled by the medium helper. */
  struct ptp_port_s *p = &state->port[args->port_index];
  p->enabled = true;
  p->link_up = true; /* updated by event handlers below */
  p->medium = args->medium;
  strncpy(p->interface_name, args->interface, sizeof(p->interface_name) - 1);
  p->interface_name[sizeof(p->interface_name) - 1] = '\0';

  int rc;
  switch (args->medium) {
  case ptp_port_medium_eth_hwts:
    rc = ptp_port_init_eth_hwts(state, args->port_index, args->interface);
    break;
  case ptp_port_medium_wifi_ftm:
    rc = ptp_port_init_wifi_ftm(state, args->port_index, args->interface);
    break;
  default:
    ptperr("bootstrap medium %d not supported\n", (int)args->medium);
    return ERROR;
  }
  if (rc != OK) {
    return ERROR;
  }

#if !SOC_EMAC_SUPPORTED
  /* No EMAC IEEE-1588 hardware — bring up the software clock backend
   * so the servo's frequency adjustment actually moves the local rate.
   * Seed with current CLOCK_REALTIME so the very first inject_sync
   * sees a familiar offset and goes through the standard jump path. */
  struct timespec sw_init_ts;
  clock_gettime(CLOCK_REALTIME, &sw_init_ts);
  if (ptp_clock_sw_init(&sw_init_ts) == 0) {
    s_use_sw_clock = true;
    ptpinfo("SW clock backend active (no EMAC IEEE-1588)\n");
  }
#endif

  /* Daemon clockIdentity is sourced from the bootstrap port's MAC.
   * EUI-64 mapping per 802.1AS-2020 §8.5.2.2 (insert 0xff:0xfe in the
   * middle of the 48-bit MAC to form the 8-byte clockIdentity). */
  uint8_t *mac = p->intf_hw_addr;
  state->own_identity.header.version = 2;
  state->own_identity.header.domain = CONFIG_ESP_PTP_DOMAIN;
  state->own_identity.header.sourceidentity[0] = mac[0];
  state->own_identity.header.sourceidentity[1] = mac[1];
  state->own_identity.header.sourceidentity[2] = mac[2];
  state->own_identity.header.sourceidentity[3] = 0xff;
  state->own_identity.header.sourceidentity[4] = 0xfe;
  state->own_identity.header.sourceidentity[5] = mac[3];
  state->own_identity.header.sourceidentity[6] = mac[4];
  state->own_identity.header.sourceidentity[7] = mac[5];
  state->own_identity.header.sourceportindex[0] = 0;
  state->own_identity.header.sourceportindex[1] = 1;
#if defined(CONFIG_ESP_PTP_SERVER) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE)
  state->own_identity.btc_priority1 = CONFIG_ESP_PTP_PRIORITY1;
  state->own_identity.btc_quality[0] = CONFIG_ESP_PTP_CLASS;
  state->own_identity.btc_quality[1] = CONFIG_ESP_PTP_ACCURACY;
  state->own_identity.btc_quality[2] = 0xff;
  state->own_identity.btc_quality[3] = 0xff;
  state->own_identity.btc_priority2 = CONFIG_ESP_PTP_PRIORITY2;
  memcpy(state->own_identity.btc_identity,
         state->own_identity.header.sourceidentity,
         sizeof(state->own_identity.btc_identity));
  state->own_identity.timesource = CONFIG_ESP_PTP_CLOCKSOURCE;
#else
  /* Statically configured as timereceiver: advertise worst priority. */
  state->own_identity.btc_priority1 = 255;
#endif

  portENTER_CRITICAL(&s_injected_lock);
  s_injected_head = s_injected_count = 0;
  for (unsigned port = 0; port < CONFIG_ESP_PTP_NUM_PORTS; ++port)
    ++s_injected_generation[port];
  s_injected_enabled = true;
  portEXIT_CRITICAL(&s_injected_lock);
  s_state = state;

  /* Run Pdelay_Req from esp_timer so the main poll loop can't starve
   * it; strict-1AS evaluators reject the resulting jitter. */
  if (ptp_is_gptp(state) && args->medium == ptp_port_medium_eth_hwts) {
    esp_timer_create_args_t timer_args = {
        .callback = pdelay_req_timer_cb,
        .arg = state,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "pdelay_req",
    };
    if (esp_timer_create(&timer_args, &s_pdelay_req_timer) == ESP_OK) {
      /* First fire in 100 ms; the callback re-arms with the current
       * next_delayreq_interval_ms after each emission. */
      esp_timer_start_once(s_pdelay_req_timer, 100 * 1000);
    } else {
      ESP_LOGW(TAG, "esp_timer_create failed for pdelay_req");
    }
  }

  ptpinfo("PTP daemon started in %s mode on port %d medium=%s\n",
          ptp_is_gptp(state) ? "gPTP" : "standard", args->port_index,
          args->medium == ptp_port_medium_eth_hwts ? "eth_hwts" : "wifi_ftm");
  return OK;
}

/* Unsubscribe multicast and destroy sockets */

static int ptp_destroy_state(FAR struct ptp_state_s *state) {
  if (s_pdelay_req_timer != NULL) {
    esp_timer_stop(s_pdelay_req_timer);
    esp_timer_delete(s_pdelay_req_timer);
    s_pdelay_req_timer = NULL;
  }
  if (state->eth_event_handler_registered) {
    esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID,
                                 ptp_eth_event_handler);
    state->eth_event_handler_registered = false;
  }

  // Remove well-known PTP multicast destination MAC addresses from the filter
  esp_eth_handle_t eth_handle;
  if (ptp_get_esp_eth_handle(state, &eth_handle) < 0) {
    ptperr("failed to get socket eth_handle %d\n", errno);
    return ERROR;
  }
  uint8_t dest_addr[ETH_ADDR_LEN];
  SET_MAC_ADDR(dest_addr, 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00);
  esp_eth_ioctl(eth_handle, ETH_CMD_DEL_MAC_FILTER, dest_addr);
  SET_MAC_ADDR(dest_addr, 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E);
  esp_eth_ioctl(eth_handle, ETH_CMD_DEL_MAC_FILTER, dest_addr);

  if (state->port[0].ptp_socket > 0) {
    close(state->port[0].ptp_socket);
    state->port[0].ptp_socket = -1;
  }
  return OK;
}

static size_t ptp_append_endpoint_decl_tlv(FAR uint8_t *msg_buf,
                                           size_t base_len);

/* Emit a Pdelay_Req beacon carrying the Endpoint Declaration TLV, addressed
 * to the gPTP bridge-group MAC regardless of the local PTP profile, so peers
 * still in gPTP can detect us after we have fallen back. The beacon is
 * informational only; no Pdelay_Resp is expected or processed.
 * Per profiles/avb_lite.md §2.3.
 */
static int ptp_send_endpoint_beacon(FAR struct ptp_state_s *state) {
  ptp_msgbuf req;
  struct timespec ts;
  size_t req_len;

  memset(&req, 0, sizeof(req));
  req.header = state->own_identity.header;
  req.header.messagetype = PTP_MSGTYPE_PDELAY_REQ | PTP_MSGTYPE_SDOID_GPTP;
  req.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
  req.header.controlfield = 5;
  req_len = sizeof(struct ptp_pdelay_req_s);
  req_len += ptp_append_endpoint_decl_tlv(req.raw, req_len);
  req.header.messagelength[1] = req_len;
  ptp_increment_sequence(&state->delay_req_seq, &req.header);

  static const uint8_t lldp_mac[6] = LLDP_MULTICAST_ADDR;
  int ret = ptp_net_send_to(state, &req, req_len, &ts, lldp_mac);
  if (ret >= 0) {
    ptpdebug("Sent endpoint beacon, seq %ld",
            (long)ptp_get_sequence(&req.header));
  }
  return ret;
}

/* Append the AVB Lite Endpoint Declaration TLV to a Pdelay-class message
 * buffer at offset base_len. Returns the number of bytes added.
 * Per profiles/avb_lite.md §2.1.
 */
_Static_assert(sizeof(struct ptp_endpoint_decl_tlv_s) == 12,
               "Endpoint Declaration TLV is 4 octets of header plus 8");

static size_t ptp_append_endpoint_decl_tlv(FAR uint8_t *msg_buf,
                                           size_t base_len) {
  FAR struct ptp_endpoint_decl_tlv_s *tlv =
      (FAR struct ptp_endpoint_decl_tlv_s *)(msg_buf + base_len);
  static const uint8_t orgid[3] = PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES;
  static const uint8_t orgsub[3] = PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES;

  tlv->type[0] = (PTP_TLV_TYPE_ORGANIZATION_EXTENSION_DO_NOT_PROPAGATE >> 8) & 0xFF;
  tlv->type[1] = PTP_TLV_TYPE_ORGANIZATION_EXTENSION_DO_NOT_PROPAGATE & 0xFF;
  tlv->length[0] = 0x00;
  tlv->length[1] = 0x08;
  memcpy(tlv->orgidentity, orgid, sizeof(orgid));
  memcpy(tlv->orgsubtype, orgsub, sizeof(orgsub));
  tlv->data = PTP_ENDPOINT_DECL_TLV_DATA;
  tlv->pad = 0; /* even lengthField, linuxptp drops odd TLVs */
  return sizeof(struct ptp_endpoint_decl_tlv_s);
}

/* Find an AVB Lite organization extension TLV by subtype in the TLVs
 * after base_len. Earlier profile revisions sent type 0x0003, current ones
 * 0x8000; both are accepted. Returns the TLV value (after its 4-octet
 * header) when lengthField is at least min_len, else NULL. */
static FAR const uint8_t *ptp_find_lite_tlv(FAR const uint8_t *msg_buf,
                                            size_t total_len, size_t base_len,
                                            const uint8_t subtype[3],
                                            uint16_t min_len) {
  static const uint8_t orgid[3] = PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES;
  size_t offset = base_len;

  while (offset + 4 <= total_len) {
    uint16_t type = ((uint16_t)msg_buf[offset] << 8) | msg_buf[offset + 1];
    uint16_t len = ((uint16_t)msg_buf[offset + 2] << 8) | msg_buf[offset + 3];
    size_t body = offset + 4;
    if (body + len > total_len) {
      break; /* malformed / truncated */
    }
    if ((type == PTP_TLV_TYPE_ORGANIZATION_EXTENSION ||
         type == PTP_TLV_TYPE_ORGANIZATION_EXTENSION_DO_NOT_PROPAGATE) &&
        len >= min_len && len >= 6 &&
        memcmp(msg_buf + body, orgid, 3) == 0 &&
        memcmp(msg_buf + body + 3, subtype, 3) == 0) {
      return msg_buf + body;
    }
    offset = body + len;
  }
  return NULL;
}

/* Scan a Pdelay-class message for the AVB Lite Endpoint Declaration TLV.
 * msg_buf points at the message start, total_len is the on-wire length,
 * base_len is sizeof the standard message body (Pdelay_Req / Resp / Fup).
 * Returns true if the TLV is present and identifies the sender as an
 * AVB Lite endpoint. Per profiles/avb_lite.md §2.1.
 */
static bool ptp_msg_has_endpoint_decl_tlv(FAR const uint8_t *msg_buf,
                                          size_t total_len, size_t base_len) {
  static const uint8_t orgsub[3] = PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES;
  FAR const uint8_t *value =
      ptp_find_lite_tlv(msg_buf, total_len, base_len, orgsub, 7);
  return value != NULL && value[6] == PTP_ENDPOINT_DECL_TLV_DATA;
}

/* Endpoint ports declare themselves in every Pdelay message; bridged
 * ports never do (profiles/avb_lite.md §2.1). */
static bool ptp_sends_endpoint_decl(FAR const struct ptp_state_s *state) {
#ifdef CONFIG_ESP_PTP_AVB_LITE_ENDPOINT
  return state->port[0].type != ptp_port_type_bridged &&
         state->port[0].medium == ptp_port_medium_eth_hwts;
#else
  (void)state;
  return false;
#endif
}

static void ptp_set_message_length(FAR struct ptp_header_s *header,
                                   size_t length) {
  header->messagelength[0] = (uint8_t)(length >> 8);
  header->messagelength[1] = (uint8_t)length;
}

/* delayAsymmetry from link speeds, profiles/avb_lite.md §5: a store and
 * forward switch holds a frame for its length at its ingress speed, so
 * with L octets 4 x L x (1/R_gm - 1/R_local), positive when the
 * timetransmitter to timereceiver direction takes longer. */
static void ptp_update_delay_asymmetry(FAR struct ptp_state_s *state) {
  int64_t asymmetry_ns = 0;
  int64_t gm_mbps = state->selected_gm_link_mbps;
  int64_t local_mbps = state->port[0].negotiated_link_mbps;
  if (!ptp_is_gptp(state) && gm_mbps > 0 && local_mbps > 0 &&
      gm_mbps != local_mbps) {
#ifdef CONFIG_ESP_PTP_LITE_PRIORITY_TAG
    const int64_t frame_octets = 66;
#else
    const int64_t frame_octets = 64;
#endif
    asymmetry_ns = 4 * frame_octets * 1000 * (local_mbps - gm_mbps) /
                   (gm_mbps * local_mbps);
  }
  if (asymmetry_ns != state->delay_asymmetry_ns) {
    state->delay_asymmetry_ns = asymmetry_ns;
    ptpinfo("Delay asymmetry %lld ns (timetransmitter %u Mbps, local %u Mbps)",
            (long long)asymmetry_ns, (unsigned)gm_mbps, (unsigned)local_mbps);
  }
}

/* Follow renegotiation of the wired link; checked every few seconds. */
static void ptp_refresh_link_speed(FAR struct ptp_state_s *state) {
  struct ptp_port_s *port = &state->port[0];
  if (port->medium != ptp_port_medium_eth_hwts || port->ptp_socket < 0) {
    return;
  }
  int64_t now_us = esp_timer_get_time();
  if (port->link_speed_checked_us != 0 &&
      now_us - port->link_speed_checked_us < 3000000) {
    return;
  }
  port->link_speed_checked_us = now_us;
  uint32_t mbps = 0;
  esp_eth_handle_t eth_handle;
  eth_speed_t speed;
  if (port->link_up && ptp_get_esp_eth_handle(state, &eth_handle) >= 0 &&
      esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &speed) == ESP_OK) {
    mbps = speed == ETH_SPEED_1000M ? 1000u
           : speed == ETH_SPEED_100M ? 100u : 10u;
  }
  if (mbps != port->negotiated_link_mbps) {
    port->negotiated_link_mbps = mbps;
    ptpinfo("port 0 negotiated link speed %u Mbps", (unsigned)mbps);
    ptp_update_delay_asymmetry(state);
  }
}

/* The EMAC snapshots RX timestamps only for frames its PTP filter
 * recognises, which need not include tagged PTP. In the AVB Lite
 * profile, where PTP may arrive priority-tagged, timestamp every
 * received frame instead. */
static void ptp_apply_rx_timestamp_mode(FAR struct ptp_state_s *state) {
#if PTPD_HAVE_ESP_ETH_CLOCK && defined(SOC_EMAC_IEEE1588V2_SUPPORTED)
  struct ptp_port_s *port = &state->port[0];
  if (port->medium != ptp_port_medium_eth_hwts || port->ptp_socket < 0) {
    return;
  }
  bool all_frames = !ptp_is_gptp(state);
  if (port->rx_timestamp_mode_set && port->rx_timestamp_all == all_frames) {
    return;
  }
  esp_eth_handle_t eth_handle;
  esp_eth_mac_t *mac = NULL;
  if (ptp_get_esp_eth_handle(state, &eth_handle) < 0 ||
      esp_eth_get_mac_instance(eth_handle, &mac) != ESP_OK || mac == NULL) {
    return;
  }
  if (esp_eth_mac_enable_ts4all(mac, all_frames) == ESP_OK) {
    port->rx_timestamp_all = all_frames;
    port->rx_timestamp_mode_set = true;
    ptpinfo("RX timestamps for %s", all_frames ? "all frames" : "PTP frames");
  }
#else
  (void)state;
#endif
}

/* Send PTP server announcement packet */

static int ptp_send_announce(FAR struct ptp_state_s *state) {
  struct ptp_announce_s msg;
  struct timespec ts;
  int ret;

  memset(&msg, 0, sizeof(msg));
  msg = state->own_identity;
  msg.header.messagetype = PTP_MSGTYPE_ANNOUNCE;
  msg.header.messagelength[1] = sizeof(msg);
  msg.header.logmessageinterval =
      msec_to_log_period(CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS);

  /* gPTP requires the PTP timescale, and AVB Lite needs one common
   * timescale for media presentation (profiles/avb_lite.md §5). */
  msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
  if (ptp_is_gptp(state)) {
    msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
  }

  ptp_increment_sequence(&state->announce_seq, &msg.header);
  ptp_gettime(state, &ts);
  timespec_to_ptp_format(&ts, msg.origintimestamp);

  /* Add the path trace TLV */
  struct ptp_pathtrace_tlv_s pathtrace_tlv;
  memset(&pathtrace_tlv, 0, sizeof(pathtrace_tlv));
  pathtrace_tlv.type[1] = 8;   // Path trace
  pathtrace_tlv.length[1] = 8; // 8 bytes
  memcpy(pathtrace_tlv.pathsequence, state->own_identity.btc_identity,
         sizeof(state->own_identity.btc_identity));
  msg.pathtracetlv = pathtrace_tlv;

  /* AVB Lite profile: announce this timetransmitter's link speed in the
   * Grandmaster Link TLV (profiles/avb_lite.md §5). Omitted while the
   * speed is unknown, so timereceivers apply no correction. */
  uint8_t wire[sizeof(msg) + sizeof(struct ptp_gm_link_tlv_s)];
  size_t wire_len = sizeof(msg);
  uint32_t link_mbps = state->port[0].negotiated_link_mbps;
  if (!ptp_is_gptp(state) && link_mbps != 0) {
    static const uint8_t orgid[3] = PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES;
    static const uint8_t orgsub[3] = PTP_GM_LINK_TLV_SUBTYPE_BYTES;
    struct ptp_gm_link_tlv_s link_tlv;
    link_tlv.type[0] = PTP_TLV_TYPE_ORGANIZATION_EXTENSION_DO_NOT_PROPAGATE >> 8;
    link_tlv.type[1] = PTP_TLV_TYPE_ORGANIZATION_EXTENSION_DO_NOT_PROPAGATE & 0xFF;
    link_tlv.length[0] = 0;
    link_tlv.length[1] = 10;
    memcpy(link_tlv.orgidentity, orgid, sizeof(orgid));
    memcpy(link_tlv.orgsubtype, orgsub, sizeof(orgsub));
    link_tlv.link_speed_mbps[0] = (uint8_t)(link_mbps >> 24);
    link_tlv.link_speed_mbps[1] = (uint8_t)(link_mbps >> 16);
    link_tlv.link_speed_mbps[2] = (uint8_t)(link_mbps >> 8);
    link_tlv.link_speed_mbps[3] = (uint8_t)link_mbps;
    memcpy(wire + sizeof(msg), &link_tlv, sizeof(link_tlv));
    wire_len += sizeof(link_tlv);
  }
  ptp_set_message_length(&msg.header, wire_len);
  memcpy(wire, &msg, sizeof(msg));

  ret = ptp_net_send(state, wire, wire_len, NULL);

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
  } else {
    ptpdebug("Sent announce, seq %ld", (long)ptp_get_sequence(&msg.header));
  }

  return ret;
}

/* Marshal a 76-byte §12.7 FollowUpInformation: 34-byte common
 * header + 10-byte preciseOriginTimestamp + 32-byte FU TLV.
 *
 * We REGENERATE rather than forward (the §11.2.13 bridge model):
 * preciseOriginTimestamp = ptpd_now(), which the wired-side servo
 * has already brought into the BTC time domain. Equivalent under
 * lock; avoids residence-time accounting. Corollary fields
 * (correctionField, cumulativeScaledRateOffset, gmTimeBaseIndicator,
 * lastGmPhaseChange, scaledLastGmFreqChange, sequenceId) are zero. */
static void
ptp_marshal_follow_up_for_beacon_ie(FAR struct ptp_state_s *state,
                                    FAR struct ptp_follow_up_s *out, int port_index) {
  memset(out, 0, sizeof(*out));

  /* Common header — start from cached own_identity template */
  out->header = state->own_identity.header;
  out->header.messagetype = PTP_MSGTYPE_FOLLOW_UP;
  out->header.controlfield = 2; /* IEEE 1588 Follow_Up controlfield */
  out->header.logmessageinterval =
      msec_to_log_period(CONFIG_ESP_PTP_SYNC_INTERVAL_MS);
  out->header.messagelength[0] = 0;
  out->header.messagelength[1] = sizeof(struct ptp_follow_up_s);
  out->header.flags[0] = 0; /* 2-step bit lives on Sync, cleared on Follow_Up */
  if (ptp_is_gptp(state)) {
    out->header.messagetype |= PTP_MSGTYPE_SDOID_GPTP;
    out->header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
  }

  out->header.sourceportindex[0] = (port_index + 1) >> 8;
  out->header.sourceportindex[1] = port_index + 1;

  /* preciseOriginTimestamp — current PTP-disciplined time */
  struct timespec ts;
  ptp_gettime(state, &ts);
  timespec_to_ptp_format(&ts, out->origintimestamp);

  /* FollowUpInformation TLV — §11.4.4.3. TLV header constants are
   * always populated; data fields are all zero by design (see
   * design-choice notes above). */
  struct ptp_info_tlv_s *tlv = (struct ptp_info_tlv_s *)out->informationtlv;
  tlv->type[1] = 3;      /* Organization extension */
  tlv->length[1] = 0x1c; /* 28 bytes of value */
  tlv->orgidentity[0] = 0x00;
  tlv->orgidentity[1] = 0x80;
  tlv->orgidentity[2] = 0xc2; /* IEEE 802.1 OUI */
  tlv->orgsubtype[2] = 1;     /* FollowUp Information per §11.4.4.3 */
  ptp_gptp_wire_normalize((uint8_t *)out, sizeof(*out), ptp_is_gptp(state));
}

/* Originate from the running hardware clock while the local clock is selected.
 * Keep this separate from received timing, so holdover is not an observation. */
static void ptp_publish_local_source(FAR struct ptp_state_s *state)
{
  bool eligible = ptp_is_gptp(state) && !s_use_sw_clock &&
      !state->selected_source_valid && state->own_identity.btc_priority1 < 255;
  if (!eligible) {
    portENTER_CRITICAL(&s_timing_lock);
    s_local_source_snapshot.valid = false;
    portEXIT_CRITICAL(&s_timing_lock);
    return;
  }
  struct timespec origin;
  if (ptp_gettime(state, &origin) != OK || origin.tv_sec < 0) return;
  ptp_timing_snapshot_t next = {
    .received_us = esp_timer_get_time(),
    .reference_ns = timespec_to_ns(&origin),
    .local_receive_ns = timespec_to_ns(&origin),
    .trim_ppb = state->freq_trim_ppb,
    .hardware_clock = true,
    .valid = true,
  };
  struct ptp_follow_up_s message;
  ptp_marshal_follow_up_for_beacon_ie(state, &message, 0);
  timespec_to_ptp_format(&origin, message.origintimestamp);
  memcpy(next.follow_up, &message, sizeof(next.follow_up));
  memcpy(next.btc_identity, state->own_identity.btc_identity, 8);
  memcpy(next.own_identity, state->own_identity.header.sourceidentity, 8);
  portENTER_CRITICAL(&s_timing_lock);
  if (!s_timing_snapshot.generation) s_timing_snapshot.generation = 1;
  next.generation = s_timing_snapshot.generation;
  s_local_source_snapshot = next;
  portEXIT_CRITICAL(&s_timing_lock);
}

/* Send PTP server synchronization packet */

static int ptp_send_sync(FAR struct ptp_state_s *state) {
  ptp_msgbuf msg; // using generic msgbuf to allow for larger follow-up size
  struct timespec ts;
  int ret;

  memset(&msg, 0, sizeof(msg));
  msg.header = state->own_identity.header;
  msg.header.messagetype = PTP_MSGTYPE_SYNC;
  msg.header.messagelength[1] = sizeof(struct ptp_sync_s);
  msg.header.logmessageinterval =
      msec_to_log_period(CONFIG_ESP_PTP_SYNC_INTERVAL_MS);

#if defined(CONFIG_ESP_PTP_TWOSTEP_SYNC) ||                              \
    defined(                                                                   \
        CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always uses two-step sync
  msg.header.flags[0] = PTP_FLAGS0_TWOSTEP;
#endif
  if (ptp_is_gptp(state)) {
    msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
    msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;   // gPTP required flag
  }

  /* Timestamp and send the sync message */

  ptp_increment_sequence(&state->sync_seq, &msg.header);
  ptp_gettime(state, &ts);
  timespec_to_ptp_format(&ts, msg.sync.origintimestamp);

  ret = ptp_net_send(state, &msg, sizeof(struct ptp_sync_s), &ts);
  if (ret < 0) {
    ptperr("sendmsg for sync message failed: %d\n", errno);
    return ret;
  }

#if defined(CONFIG_ESP_PTP_TWOSTEP_SYNC) ||                              \
    defined(                                                                   \
        CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always uses two-step sync

  /* Send the follow up message */

  timespec_to_ptp_format(&ts, msg.follow_up.origintimestamp);
  msg.header.messagetype = PTP_MSGTYPE_FOLLOW_UP;
  msg.header.messagelength[1] = sizeof(struct ptp_follow_up_s);
  msg.header.flags[0] = 0;     // Reset 2-step flag
  msg.header.controlfield = 2; // Follow-up message

  if (ptp_is_gptp(state)) {
    msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
  }

  /* Add the information TLV (required for gPTP and ignored otherwise) */

  struct ptp_info_tlv_s info_tlv;
  memset(&info_tlv, 0, sizeof(info_tlv));
  info_tlv.type[1] = 3;                       // Organization extension
  info_tlv.length[1] = 0x1c;                  // 28 bytes
  uint8_t orgidentity[] = {0x00, 0x80, 0xc2}; // 32962 (gPTP required value)
  memcpy(info_tlv.orgidentity, orgidentity, sizeof(orgidentity));
  info_tlv.orgsubtype[2] = 1; // gPTP required value
  memcpy(msg.follow_up.informationtlv, &info_tlv, sizeof(info_tlv));

  ret = ptp_net_send(state, &msg, sizeof(struct ptp_follow_up_s), NULL);
  if (ret < 0) {
    ptperr("sendto for follow-up message failed: %d\n", errno);
    return ret;
  }

  ptpdebug("Sent sync + follow-up, seq %ld",
          (long)ptp_get_sequence(&msg.header));
#else
  ptpdebug("Sent sync, seq %ld", (long)ptp_get_sequence(&msg.header));
#endif /* CONFIG_ESP_PTP_TWOSTEP_SYNC */

  return OK;
}

/* Unicast Delay_Req, IEEE 1588-2019 16.9 and profiles/avb_lite.md §5:
 * address the timetransmitter the selected Announce came from, and go
 * back to multicast for a while after three unanswered requests. */
#define PTP_UNICAST_DELAY_REQ_MISSES 3
#define PTP_UNICAST_DELAY_REQ_RETRY_US (60 * 1000000LL)

static void ptp_reset_unicast_delay_req(FAR struct ptp_state_s *state) {
  state->unicast_delay_req_outstanding = false;
  state->unicast_delay_req_misses = 0;
  state->unicast_delay_req_retry_us = 0;
}

static bool ptp_unicast_delay_req_active(FAR struct ptp_state_s *state) {
#ifdef CONFIG_ESP_PTP_UNICAST_DELAY_REQ
  if (!state->selected_source_mac_valid ||
      (state->selected_source_mac[0] & 0x01)) {
    return false;
  }
  int64_t now_us = esp_timer_get_time();
  if (state->unicast_delay_req_outstanding) {
    state->unicast_delay_req_outstanding = false;
    if (++state->unicast_delay_req_misses >= PTP_UNICAST_DELAY_REQ_MISSES) {
      state->unicast_delay_req_misses = 0;
      state->unicast_delay_req_retry_us = now_us + PTP_UNICAST_DELAY_REQ_RETRY_US;
      ptpwarn("Unicast Delay_Req unanswered, sending multicast");
    }
  }
  if (state->unicast_delay_req_retry_us != 0) {
    if (now_us < state->unicast_delay_req_retry_us) {
      return false;
    }
    state->unicast_delay_req_retry_us = 0;
    ptpinfo("Retrying unicast Delay_Req");
  }
  return true;
#else
  (void)state;
  return false;
#endif
}

/* Send delay request packet to selected source */

static int ptp_send_delay_req(FAR struct ptp_state_s *state) {
#if CONFIG_ESP_PTP_PEER_LOSS_TEST
  static bool testing;
  int64_t test_time = esp_timer_get_time();
  bool suppress = test_time >= 60000000LL && test_time < 66000000LL;
  if (testing != suppress) {
    testing = suppress;
    ptpinfo("PDELAY_LOSS_TEST,%u", suppress);
  }
  if (suppress) return OK;
#endif
  ptp_msgbuf req;
  int ret;
  size_t req_len;

  memset(&req, 0, sizeof(req));
  req.header = state->own_identity.header;
  /* gPTP: 0x7F "no change / unspecified" sentinel per 802.1AS
   * §11.4.5.3 + IEEE 1588-2008 §7.7.2.1. */
  if (ptp_is_gptp(state)) {
    req.header.logmessageinterval = 0x7F;
  } else {
    req.header.logmessageinterval =
        msec_to_log_period(state->port[0].delayreq_interval_ms);
  }

  bool unicast_request = false;
  if (ptp_is_gptp(state)) {
    req.header.messagetype = PTP_MSGTYPE_PDELAY_REQ | PTP_MSGTYPE_SDOID_GPTP;
    req.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
    req.header.controlfield = 5;
    req_len = sizeof(struct ptp_pdelay_req_s);
    if (ptp_sends_endpoint_decl(state)) {
      req_len += ptp_append_endpoint_decl_tlv(req.raw, req_len);
    }

  } else {
    req.header.messagetype = PTP_MSGTYPE_DELAY_REQ;
    ptp_gettime(state, &state->port[0].delayreq_time);
    timespec_to_ptp_format(&state->port[0].delayreq_time,
                           req.delay_req.origintimestamp);
    req_len = sizeof(struct ptp_delay_req_s);
    unicast_request = ptp_unicast_delay_req_active(state);
    if (unicast_request) {
      req.header.flags[0] |= PTP_FLAGS0_UNICAST;
    }
  }
  ptp_set_message_length(&req.header, req_len);

  ptp_increment_sequence(&state->delay_req_seq, &req.header);

  const bool peer_request = ptp_is_gptp(state);
  struct timespec transmit_time = {.tv_nsec = -1};
  uint32_t request_generation = 0;
  unsigned expired_losses = 0;
  bool missing_follow_up = false;
  unsigned expired_sequence = 0, response_rejections = 0, response_count = 0;
  int64_t first_response_us = 0, published_us = 0;
  unsigned follow_up_ingress = 0, follow_up_count = 0, follow_up_rejections = 0;
  if (peer_request) {
    int64_t deadline_us = esp_timer_get_time() +
        (int64_t)state->port[0].delayreq_interval_ms * 1000;
    portENTER_CRITICAL(&s_peer_lock);
    if (ptp_peer_expire(&state->port[0].peer_exchange, esp_timer_get_time())) {
      expired_losses = state->port[0].peer_exchange.lost_responses;
      missing_follow_up = state->port[0].peer_exchange.response_seen;
      expired_sequence = state->port[0].peer_exchange.sequence;
      response_rejections = state->port[0].peer_exchange.response_rejections;
      response_count = state->port[0].peer_exchange.response_count;
      first_response_us = state->port[0].peer_exchange.first_response_us;
      published_us = state->port[0].peer_exchange.published_us;
      follow_up_ingress = s_peer_ingress_count;
      follow_up_count = state->port[0].peer_exchange.follow_up_count;
      follow_up_rejections = state->port[0].peer_exchange.follow_up_rejections;
    }
    s_peer_ingress_sequence = ptp_get_sequence(&req.header);
    s_peer_ingress_count = 0;
    request_generation = ptp_peer_begin(&state->port[0].peer_exchange,
        ptp_get_sequence(&req.header), &req.header, deadline_us);
    portEXIT_CRITICAL(&s_peer_lock);
  }
  if (expired_losses) {
    ptpinfo("PDELAY_LOST,%u,%u", expired_losses, missing_follow_up);
    ptpinfo("PDELAY_LOST_DETAIL,%u,%u,%u,%lld,%lld", expired_sequence,
            response_rejections, response_count, (long long)first_response_us,
            (long long)published_us);
    ptpinfo("PDELAY_LOST_FU,%u,%u,%u,%u", expired_sequence,
            follow_up_ingress, follow_up_count, follow_up_rejections);
  }
  if (unicast_request) {
    ret = ptp_net_send_to(state, &req, req_len, &state->port[0].delayreq_time,
                          state->selected_source_mac);
    if (ret > 0) {
      state->unicast_delay_req_outstanding = true;
    }
  } else {
    ret = ptp_net_send(state, &req, req_len,
                       peer_request ? &transmit_time : &state->port[0].delayreq_time);
  }
  if (peer_request) {
    portENTER_CRITICAL(&s_peer_lock);
    if (ret <= 0 || !ptp_peer_publish(&state->port[0].peer_exchange, request_generation,
                                     &transmit_time, esp_timer_get_time()))
      ptp_peer_cancel(&state->port[0].peer_exchange, request_generation);
    portEXIT_CRITICAL(&s_peer_lock);
  }

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
  } else {
    clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_transmitted_delayreq);
    ptpdebug("Sent delay req, seq %ld", (long)ptp_get_sequence(&req.header));
  }

  return ret;
}

/* Timer callback: send one Pdelay_Req then re-arm at the current
 * next_delayreq_interval_ms. Runs in esp_timer task (priority 22, in
 * IRAM) so the cadence is independent of PTPD main-loop scheduling. */
static IRAM_ATTR void pdelay_req_timer_cb(void *arg) {
  FAR struct ptp_state_s *state = (FAR struct ptp_state_s *)arg;
  if (state == NULL || state->stop) {
    return;
  }
  /* Only emit Pdelay_Req if conditions match the original loop check. */
  if (state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0 &&
      (ptp_is_gptp(state) ||
       (state->selected_source_valid && state->port[0].can_send_delayreq))) {
    /* Record lateness against the planned interval for the diagnostic. */
    if (state->port[0].last_transmitted_delayreq.tv_sec != 0) {
      struct timespec time_now;
      struct timespec delta;
      clock_gettime(CLOCK_MONOTONIC, &time_now);
      clock_timespec_subtract(
          &time_now, &state->port[0].last_transmitted_delayreq, &delta);
      int64_t late_ms = (int64_t)timespec_to_ms(&delta) -
                        (int64_t)state->port[0].next_delayreq_interval_ms;
      if (late_ms > 0) {
        ptpd_lateness_record_tx(late_ms * 1000LL);
      }
    }
    ptp_send_delay_req(state);
  }
  /* Re-arm with the current interval (updated when Pdelay_Resp arrives). */
  uint32_t interval_us =
      (uint32_t)state->port[0].next_delayreq_interval_ms * 1000U;
  if (interval_us < 100000U) {
    interval_us = 1000000U; /* fallback if uninitialised */
  }
  esp_timer_start_once(s_pdelay_req_timer, interval_us);
}

static void ptp_check_profile_fallback(FAR struct ptp_state_s *state) {
#ifndef CONFIG_ESP_PTP_AVB_LITE_ENDPOINT
  (void)state;
  return;
#endif
  if (!ptp_is_gptp(state) || state->gptp_fallback_done) {
    return;
  }

  /* AVB Lite fallback is endpoint-only (profiles/avb_lite.md §2:
   * "An AVB endpoint that also supports AVB Lite must automatically
   * fall back..."). A bridge port relays timing between AVB domains
   * and must remain in gPTP regardless of transient Pdelay losses. */
  if (state->port[0].type == ptp_port_type_bridged) {
    return;
  }

  /* AVB Lite fallback condition 1 (profiles/avb_lite.md §2.2):
   * a Pdelay_{Req,Resp,Resp_Follow_Up} arrived carrying the Endpoint
   * Declaration TLV — the peer is an endpoint, no AVB-aware bridge between us.
   */

  if (state->port[0].peer_is_endpoint) {
    ptpwarn("Endpoint Declaration TLV seen on Pdelay channel; "
            "switching to standard PTP mode\n");
    state->gptp_fallback_done = true;
    state->avb_lite_fallback_reason = 1;
    state->active_ptp_profile = ptp_profile_standard;
    ptp_reset_for_profile(state);
    return;
  }

  /* AVB Lite policy: nine expired, incomplete peer exchanges. Count only
   * published requests after their deadline, not the request still in flight.
   * This policy is separate from gPTP asCapable fault tolerance. */

  portENTER_CRITICAL(&s_peer_lock);
  ptp_peer_expire(&state->port[0].peer_exchange, esp_timer_get_time());
  unsigned unanswered = state->port[0].peer_exchange.lost_responses;
  portEXIT_CRITICAL(&s_peer_lock);
  if (unanswered >= 9) {
    ptpwarn(
        "Incomplete peer exchanges after %u deadlines; switching to standard PTP mode\n",
        unanswered);
    state->gptp_fallback_done = true;
    state->avb_lite_fallback_reason = 2;
    state->active_ptp_profile = ptp_profile_standard;
    ptp_reset_for_profile(state);
    return;
  }

  /* AVB Lite fallback condition 3 (profiles/avb_lite.md §2.2):
   * Pdelay_Resp from two or more distinct sourceidentity values, indicating
   * a flooding non-AVB switch in the L2 path rather than a single AVB
   * boundary-clock peer. */

  if (state->port[0].pdelay_multi_responder) {
    ptpwarn(
        "Pdelay_Resp from multiple sources; switching to standard PTP mode\n");
    state->gptp_fallback_done = true;
    state->avb_lite_fallback_reason = 3;
    state->active_ptp_profile = ptp_profile_standard;
    ptp_reset_for_profile(state);
  }
}

/* Radio completion records contain values only, never daemon pointers. */
static void ptp_wifi_capable_completions(struct ptp_state_s *state) {
  ptp_wifi_capable_result_t result;
  for (unsigned count = 0; count < PTP_WIFI_PEERS_MAX * CONFIG_ESP_PTP_NUM_PORTS &&
       ptp_wifi_capable_result(&result); ++count) {
    if (result.port_index < 0 || result.port_index >= CONFIG_ESP_PTP_NUM_PORTS ||
        result.generation != ptp_wifi_link_generation(result.port_index)) continue;
    portENTER_CRITICAL(&s_peer_lock);
    struct ptp_port_s *port = &state->port[result.port_index];
    if (port->wifi_mode == ptp_port_wifi_mode_ap)
      ptp_wifi_peers_sent(&port->wifi_peers, result.destination, result.association,
          result.token, result.accepted, result.completed_us);
    else if (port->wifi_mode == ptp_port_wifi_mode_sta)
      ptp_wifi_neighbor_sent(&port->wifi_neighbor, result.destination, result.association,
          result.token, result.accepted, result.completed_us);
    portEXIT_CRITICAL(&s_peer_lock);
  }
}

static void ptp_wifi_ap_capable_send(struct ptp_state_s *state, int index, bool enabled) {
  struct ptp_port_s *port = &state->port[index];
  for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
    /* Snapshot generation before association state, so a concurrent event retires work. */
    uint32_t generation = ptp_wifi_link_generation(index);
    int64_t now_us = esp_timer_get_time();
    uint8_t destination[6], source_port[10], target[10], message[PTP_CAPABLE_MESSAGE_LENGTH];
    portENTER_CRITICAL(&s_peer_lock);
    ptp_wifi_peer_t *peer = &port->wifi_peers.entries[slot];
    bool publishable = ptp_wifi_peers_publishable(&port->wifi_peers);
    uint32_t token = ptp_capable_schedule_begin(&peer->transmit,
                                               enabled && publishable && peer->associated, now_us);
    uint32_t association = peer->association;
    uint16_t sequence = peer->transmit.sequence;
    int8_t interval = peer->transmit.advertised_interval;
    bool bound = peer->bound;
    memcpy(target, peer->port_identity, 10);
    memcpy(destination, peer->mac, 6);
    portEXIT_CRITICAL(&s_peer_lock);
    if (!token) continue;
    memcpy(source_port, state->own_identity.header.sourceidentity, 8);
    uint16_t logical_port = ptp_wifi_association_port(index + 1, CONFIG_ESP_PTP_NUM_PORTS, slot);
    source_port[8] = logical_port >> 8;
    source_port[9] = logical_port;
#ifdef CONFIG_ESP_PTP_INTERVAL_PROBE
    if (enabled && bound && publishable) ptp_wifi_interval_probe_start(index, true, port->intf_hw_addr,
        source_port, destination, target, generation);
#else
    (void)bound;
#endif
    size_t length = ptp_signaling_write_capable(message, sizeof(message), source_port,
        CONFIG_ESP_PTP_DOMAIN, sequence, interval);
    if (!length || ptp_wifi_send_capable_peer(index, true, port->intf_hw_addr, destination,
          generation, association, token, message, length) != 0) {
      portENTER_CRITICAL(&s_peer_lock);
      ptp_wifi_peers_sent(&port->wifi_peers, destination, association, token, false, now_us);
      portEXIT_CRITICAL(&s_peer_lock);
    }
  }
}

static void ptp_wifi_sta_capable_send(struct ptp_state_s *state, int index, bool enabled) {
  struct ptp_port_s *port = &state->port[index];
  uint32_t generation = ptp_wifi_link_generation(index);
  int64_t now_us = esp_timer_get_time();
  uint8_t destination[6], target[10], source_port[10], message[PTP_CAPABLE_MESSAGE_LENGTH];
  portENTER_CRITICAL(&s_peer_lock);
  ptp_wifi_neighbor_t *neighbor = &port->wifi_neighbor;
  uint32_t token = ptp_capable_schedule_begin(&neighbor->transmit,
                                             enabled && neighbor->associated, now_us);
  uint32_t association = neighbor->association;
  uint16_t sequence = neighbor->transmit.sequence;
  int8_t interval = neighbor->transmit.advertised_interval;
  bool bound = neighbor->bound;
  memcpy(destination, neighbor->bssid, 6);
  memcpy(target, neighbor->port_identity, 10);
  portEXIT_CRITICAL(&s_peer_lock);
  memcpy(source_port, state->own_identity.header.sourceidentity, 8);
  source_port[8] = (index + 1) >> 8;
  source_port[9] = index + 1;
#ifdef CONFIG_ESP_PTP_INTERVAL_PROBE
  if (enabled && bound) ptp_wifi_interval_probe_start(index, false, port->intf_hw_addr,
      source_port, destination, target, generation);
#else
  (void)bound;
#endif
  if (!token) return;
  size_t length = ptp_signaling_write_capable(message, sizeof(message), source_port,
      CONFIG_ESP_PTP_DOMAIN, sequence, interval);
  if (!length || ptp_wifi_send_capable_peer(index, false, port->intf_hw_addr, destination,
        generation, association, token, message, length) != 0) {
    portENTER_CRITICAL(&s_peer_lock);
    ptp_wifi_neighbor_sent(&port->wifi_neighbor, destination, association, token, false, now_us);
    portEXIT_CRITICAL(&s_peer_lock);
  }
}

static void ptp_wired_capable_send(struct ptp_state_s *state) {
  struct ptp_port_s *port = &state->port[0];
  int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_peer_lock);
  uint32_t lifecycle = port->peer_exchange.lifecycle;
  const uint8_t *identity = port->peer_rate.anchored &&
      port->peer_rate.lifecycle == lifecycle ? port->peer_rate.responder : NULL;
  ptp_wired_capable_bind(&port->wired_capable, lifecycle, identity);
  bool enabled = ptp_is_gptp(state) && port->enabled && port->link_up && port->ptp_socket >= 0;
  uint32_t token = ptp_capable_schedule_begin(&port->wired_capable.transmit, enabled, now_us);
  uint16_t sequence = port->wired_capable.transmit.sequence;
  int8_t interval = port->wired_capable.transmit.advertised_interval;
  portEXIT_CRITICAL(&s_peer_lock);
  if (!token) return;
  uint8_t source_port[10], message[PTP_CAPABLE_MESSAGE_LENGTH];
  memcpy(source_port, state->own_identity.header.sourceidentity, 8);
  memcpy(source_port + 8, state->own_identity.header.sourceportindex, 2);
  size_t length = ptp_signaling_write_capable(message, sizeof(message), source_port,
      CONFIG_ESP_PTP_DOMAIN, sequence, interval);
  bool accepted = length && ptp_net_send(state, message, length, NULL) > 0;
  portENTER_CRITICAL(&s_peer_lock);
  if (port->peer_exchange.lifecycle == lifecycle)
    ptp_capable_schedule_finish(&port->wired_capable.transmit, token, accepted,
                                esp_timer_get_time());
  portEXIT_CRITICAL(&s_peer_lock);
}

/* Check if we need to send packets */

static int ptp_periodic_send(FAR struct ptp_state_s *state) {
  /* Capability discovery continues while the physical peer is unqualified. */
  if (state->port[0].medium == ptp_port_medium_eth_hwts)
    ptp_wired_capable_send(state);

  /* Wireless discovery uses unicast and never waits for a radio RPC here. */
  ptp_wifi_capable_completions(state);
  for (int index = 0; index < CONFIG_ESP_PTP_NUM_PORTS; ++index) {
    struct ptp_port_s *port = &state->port[index];
    if (port->medium != ptp_port_medium_wifi_ftm) continue;
    bool enabled = ptp_is_gptp(state) && port->enabled && port->link_up &&
        (port->wifi_mode == ptp_port_wifi_mode_ap ||
         port->wifi_mode == ptp_port_wifi_mode_sta);
    if (port->wifi_mode == ptp_port_wifi_mode_ap) {
      ptp_wifi_ap_capable_send(state, index, enabled);
      continue;
    }
    ptp_wifi_sta_capable_send(state, index, enabled);
  }

#if defined(CONFIG_ESP_PTP_SERVER) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE)
  /* If there is no better timetransmitter clock on the network,
   * act as the reference source and send server packets. */

  /* "I am BTC" Announce + Sync via port[0]'s L2TAP socket; only valid
   * on an eth_hwts port. wifi_ftm uses the egress-cb path below. */
  if (!state->selected_source_valid && state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0 &&
      (!ptp_is_gptp(state) || ptp_wired_capability(state))) {
    struct timespec time_now;
    struct timespec delta;

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now,
                            &state->port[0].last_transmitted_announce, &delta);
    if (timespec_to_ms(&delta) > CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS) {
      state->port[0].last_transmitted_announce = time_now;
      ptp_send_announce(state);
    }

    clock_timespec_subtract(&time_now, &state->port[0].last_transmitted_sync,
                            &delta);
    if (timespec_to_ms(&delta) > CONFIG_ESP_PTP_SYNC_INTERVAL_MS &&
        (!ptp_is_gptp(state) || state->own_identity.btc_priority1 < 255)) {
      /* Advance on the nominal grid rather than from the send time, so
       * the poll granularity does not stretch the mean interval. Snap
       * to now when far behind (first Sync, long stall). */
      if (timespec_to_ms(&delta) > 2 * CONFIG_ESP_PTP_SYNC_INTERVAL_MS) {
        state->port[0].last_transmitted_sync = time_now;
      } else {
        const struct timespec interval = {
            .tv_sec = CONFIG_ESP_PTP_SYNC_INTERVAL_MS / 1000,
            .tv_nsec = (CONFIG_ESP_PTP_SYNC_INTERVAL_MS % 1000) * 1000000L};
        clock_timespec_add(&state->port[0].last_transmitted_sync, &interval,
                           &state->port[0].last_transmitted_sync);
      }
      ptp_send_sync(state);
    }
  }
#endif /* CONFIG_ESP_PTP_SERVER */

  /* Sync emission on wifi_ftm ports via the §12.7 FollowUpInformation
   * IE. Runs regardless of selected_source_valid (bridge republishes
   * upstream; BTC publishes own). */
  for (int p = 0; p < CONFIG_ESP_PTP_NUM_PORTS; p++) {
    struct ptp_port_s *port = &state->port[p];
    if (!port->enabled)
      continue;
    if (ptp_is_gptp(state) &&
        (state->selected_source_valid ? state->selected_source.btc_priority1 :
         state->own_identity.btc_priority1) == 255)
      continue;
    if (port->medium != ptp_port_medium_wifi_ftm)
      continue;
    if (port->sync_egress_cb == NULL)
      continue;

    struct timespec time_now, delta;
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now, &port->last_transmitted_sync, &delta);
    if (timespec_to_ms(&delta) <= CONFIG_ESP_PTP_SYNC_INTERVAL_MS) {
      continue;
    }
    port->last_transmitted_sync = time_now;

    struct ptp_follow_up_s fu;
    ptp_marshal_follow_up_for_beacon_ie(state, &fu, p);
    port->sync_egress_cb(p, (const uint8_t *)&fu, sizeof(fu),
                         port->sync_egress_ctx);
  }

  /* §12.2 Announce emission on wifi_ftm/AP ports. The bridge re-emits
   * the selected upstream BTC's Announce (or own_identity if we ARE
   * the BTC) as unicast to each associated STA so the STA's BMCA path
   * sees real GM priority / clockQuality (§12.7 IE only carries Sync
   * timing). Cadence matches the wired Announce interval. */
  for (int p = 0; p < CONFIG_ESP_PTP_NUM_PORTS; p++) {
    struct ptp_port_s *port = &state->port[p];
    if (!port->enabled)
      continue;

    if (port->medium != ptp_port_medium_wifi_ftm)
      continue;
    if (port->wifi_mode != ptp_port_wifi_mode_ap)
      continue;

    struct timespec time_now, delta;
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now, &port->last_transmitted_announce,
                            &delta);
    /* Refresh the shared body faster than per-peer transmission deadlines. */
    int publish_ms = CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS / 4;
    if (publish_ms < 1) publish_ms = 1;
    if (publish_ms > 125) publish_ms = 125;
    if (timespec_to_ms(&delta) < publish_ms) continue;
    port->last_transmitted_announce = time_now;

    /* Build Announce body. Prefer the selected upstream source's
     * identity/quality if we have one (bridge relay case); otherwise
     * publish our own (we are the BTC). Matches ptp_send_announce's
     * layout (header + body + path-trace TLV). */
    struct ptp_announce_s msg;
    memset(&msg, 0, sizeof(msg));
    if (state->selected_source_valid) {
      msg = state->selected_source;
      unsigned steps = ((unsigned)msg.stepsremoved[0] << 8) | msg.stepsremoved[1];
      if (steps == 65535) continue;
      ++steps;
      msg.stepsremoved[0] = steps >> 8;
      msg.stepsremoved[1] = steps;
    } else {
      msg = state->own_identity;
    }
    memcpy(msg.header.sourceidentity, state->own_identity.header.sourceidentity, 8);
    msg.header.sourceportindex[0] = (p + 1) >> 8;
    msg.header.sourceportindex[1] = p + 1;
    msg.header.messagetype = PTP_MSGTYPE_ANNOUNCE;
    msg.header.messagelength[1] = sizeof(msg);
    msg.header.logmessageinterval =
        msec_to_log_period(CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS);
    if (ptp_is_gptp(state)) {
      msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP;
      msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
    }
    /* The worker assigns each logical port its own transmit sequence. */
    struct timespec origin_ts;
    ptp_gettime(state, &origin_ts);
    timespec_to_ptp_format(&origin_ts, msg.origintimestamp);
    uint8_t wire[PTP_ANNOUNCE_MAX_LENGTH] = {0};
    _Static_assert(offsetof(struct ptp_announce_s, pathtracetlv) == PTP_ANNOUNCE_BODY_LENGTH,
                   "Announce body layout changed");
    memcpy(wire, &msg, PTP_ANNOUNCE_BODY_LENGTH);
    ptp_path_trace_t empty_path = {0};
    const ptp_path_trace_t *path = state->selected_source_valid
                                      ? &state->selected_path : &empty_path;
    size_t tlv_length = 0;
    bool trace_known = !state->selected_source_valid || path->count;
#ifdef CONFIG_ESP_PTP_ANNOUNCE_PATH_PROBE
    static int64_t path_probe_start[CONFIG_ESP_PTP_NUM_PORTS];
    static unsigned path_probe_phase[CONFIG_ESP_PTP_NUM_PORTS];
    int64_t probe_now = esp_timer_get_time();
    if (!path_probe_start[p] && state->selected_source_valid) {
      path_probe_start[p] = probe_now;
      ptpinfo("ANNPATH,%d,0", p);
    }
    if (path_probe_start[p]) {
      int64_t elapsed = probe_now - path_probe_start[p];
      unsigned phase = elapsed < 30000000 ? 0 : elapsed < 38000000 ? 1 : 2;
      if (phase != path_probe_phase[p]) {
        path_probe_phase[p] = phase;
        ptpinfo("ANNPATH,%d,%u", p, phase);
      }
      if (phase == 1) trace_known = false;
    }
#endif
    /* An unknown upstream path stays unknown through this relay. */
    if (trace_known) {
      tlv_length = ptp_path_trace_write(
          wire + PTP_ANNOUNCE_BODY_LENGTH, sizeof(wire) - PTP_ANNOUNCE_BODY_LENGTH,
          path, state->own_identity.header.sourceidentity);
      if (!tlv_length) continue;
    }
    size_t wire_length = PTP_ANNOUNCE_BODY_LENGTH + tlv_length;
    wire[2] = wire_length >> 8;
    wire[3] = wire_length;
    ptp_gptp_wire_normalize(wire, wire_length, ptp_is_gptp(state));
    ptp_wifi_ap_send_announce(p, port->intf_hw_addr, wire, wire_length);
  }

  /* Post-fallback endpoint beacon (profiles/avb_lite.md §2.3): after
   * fallback to standard PTP, periodically emit Pdelay_Req-with-TLV to
   * the gPTP bridge-group MAC so gPTP peers can detect us and follow.
   * 3 s cadence matches the §2.2 evaluation window. */

  /* Endpoint-fallback beacon — only on eth_hwts with a socket. */
  if (state->gptp_fallback_done && state->active_ptp_profile == ptp_profile_standard &&
      state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0) {
    struct timespec time_now;
    struct timespec delta;
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now, &state->port[0].last_endpoint_beacon,
                            &delta);
    if (state->port[0].last_endpoint_beacon.tv_sec == 0 ||
        timespec_to_ms(&delta) >= 3000) {
      ptp_send_endpoint_beacon(state);
      state->port[0].last_endpoint_beacon = time_now;
    }
  }

  return OK;
}

/* Process received PTP announcement */

static int ptp_process_announce(FAR struct ptp_state_s *state,
                                FAR struct ptp_announce_s *msg, size_t length) {
  ptp_path_trace_t path;
  if (!ptp_path_trace_parse((const uint8_t *)msg, length,
                            state->own_identity.header.sourceidentity, &path))
    return OK;
  unsigned steps = ((unsigned)msg->stepsremoved[0] << 8) | msg->stepsremoved[1];
  if (ptp_is_gptp(state) &&
      (steps >= 255 || !memcmp(msg->header.sourceidentity,
                               state->own_identity.header.sourceidentity, 8)))
    return OK;
  if (state->port[0].medium == ptp_port_medium_wifi_ftm &&
      state->port[0].wifi_mode == ptp_port_wifi_mode_sta &&
      state->port[0].rx_source_mac_valid) {
    uint8_t identity[10];
    memcpy(identity, msg->header.sourceidentity, 8);
    memcpy(identity + 8, msg->header.sourceportindex, 2);
    portENTER_CRITICAL(&s_peer_lock);
    bool changed_neighbor = !state->port[0].wifi_neighbor.bound ||
        memcmp(state->port[0].wifi_neighbor.port_identity, identity, 10);
    bool bound = ptp_wifi_neighbor_bind(&state->port[0].wifi_neighbor,
        state->port[0].rx_association, state->port[0].rx_source_mac, identity);
    if (bound && memcmp(state->port[0].capable_receive.message.source_port, identity, 10))
      state->port[0].capable_receive.valid = false;
    portEXIT_CRITICAL(&s_peer_lock);
    if (!bound) return OK;
    if (changed_neighbor)
      ptpinfo("WIFI_NEIGHBOR,%u", state->port[0].rx_association);
  }
  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_announce);
  bool same_port = !memcmp(msg->header.sourceidentity,
                          state->selected_source.header.sourceidentity, 8) &&
                   !memcmp(msg->header.sourceportindex,
                           state->selected_source.header.sourceportindex, 2);
  bool same_root = !memcmp(msg->btc_identity, state->selected_source.btc_identity, 8);
  if (!is_better_clock(msg, &state->own_identity)) {
    if (same_port) {
      ptp_invalidate_timing();
      state->selected_source_valid = false;
      memset(&state->selected_source, 0, sizeof(state->selected_source));
      memset(&state->selected_path, 0, sizeof(state->selected_path));
    }
    return OK;
  }
  if (!same_port && state->selected_source_valid &&
      !is_better_clock(msg, &state->selected_source))
    return OK;

  bool changed = !state->selected_source_valid || !same_port || !same_root;
  bool vector_changed = memcmp(&msg->btc_priority1, &state->selected_source.btc_priority1, 14) ||
      memcmp(msg->stepsremoved, state->selected_source.stepsremoved, 2);
  if (changed || vector_changed)
    ptp_sync_receipt_start(&state->port[0].sync_receipt, esp_timer_get_time(),
        INT64_C(3000) * CONFIG_ESP_PTP_SYNC_INTERVAL_MS);
  bool path_changed = state->selected_path.count != path.count ||
                      memcmp(state->selected_path.identities, path.identities,
                             path.count * 8);
  if (changed) {
    ptpinfo("Switching PTP time source\n");
    ptp_invalidate_timing();
    state->port[0].twostep_pending = false;
    state->port[0].last_received_sync = state->port[0].last_received_announce;
    state->port[0].path_delay_avgcount = 0;
    state->port[0].path_delay_ns = 0;
    state->correction_ns = 0;
    state->port[0].delayreq_time.tv_sec = 0;
  }
  memset(&state->selected_source, 0, sizeof(state->selected_source));
  memcpy(&state->selected_source, msg, PTP_ANNOUNCE_BODY_LENGTH);
  state->selected_path = path;

  uint32_t gm_link_mbps = 0;
  if (!ptp_is_gptp(state)) {
    static const uint8_t gm_link_subtype[3] = PTP_GM_LINK_TLV_SUBTYPE_BYTES;
    FAR const uint8_t *value = ptp_find_lite_tlv(
        (FAR const uint8_t *)msg, length, PTP_ANNOUNCE_BODY_LENGTH,
        gm_link_subtype, 10);
    if (value != NULL) {
      gm_link_mbps = ((uint32_t)value[6] << 24) | ((uint32_t)value[7] << 16) |
                     ((uint32_t)value[8] << 8) | value[9];
    }
  }
  state->selected_gm_link_mbps = gm_link_mbps;
  ptp_update_delay_asymmetry(state);

  if (state->port[0].rx_source_mac_valid &&
      (!state->selected_source_mac_valid ||
       memcmp(state->selected_source_mac, state->port[0].rx_source_mac, 6))) {
    memcpy(state->selected_source_mac, state->port[0].rx_source_mac, 6);
    state->selected_source_mac_valid = true;
    ptp_reset_unicast_delay_req(state);
  }
  if (changed || path_changed) {
    ptpinfo("Selected Announce path: %u clock(s)", path.count);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, path.identities, path.count * 8, ESP_LOG_INFO);
  }
  state->last_selected_announce = state->port[0].last_received_announce;
  return OK;
}

static void ptp_lock_local_clock_freq(FAR struct ptp_state_s *state,
                                      FAR struct timespec *remote_timestamp,
                                      FAR struct timespec *local_timestamp) {
  // Compute how off we are against the timetransmitter
  int64_t offset_ns = timespec_delta_ns(remote_timestamp, local_timestamp);
  if (ptp_is_gptp(state)) {
    offset_ns += state->port[0].peer_delay_ns + state->correction_ns;
  } else {
    offset_ns += state->port[0].path_delay_ns + state->delay_asymmetry_ns;
  }

  /* The ESP-IDF hardware clock applies ADJ_FREQUENCY relative to its current
   * addend. Keep the PI output as an absolute trim and apply only its delta;
   * applying the complete output every Sync accumulates rate error. */
  /* Scale wired integration by elapsed local time. The remote/local
   * interval difference remains diagnostic, not a frequency estimate. */
  int64_t remote_time_ns = timespec_to_ns(remote_timestamp);
  int64_t local_time_ns = timespec_to_ns(local_timestamp);
  int64_t remote_delta_ns = remote_time_ns - state->remote_time_ns_prev;
  int64_t local_delta_ns = local_time_ns - state->local_time_ns_prev;
  int64_t tick_diff = remote_delta_ns - local_delta_ns;

  int64_t target_trim;
  int64_t saved_integral = state->offset_pi.wired_integral_q16;
  if (s_use_sw_clock) {
    /* Keep the existing controller for noisy beacon observations. */
    state->offset_pi.drift_acc += offset_ns / PTP_FREQ_I_DIV_SW;
    if (state->offset_pi.drift_acc > ADJ_FREQ_MAX) state->offset_pi.drift_acc = ADJ_FREQ_MAX;
    if (state->offset_pi.drift_acc < -ADJ_FREQ_MAX) state->offset_pi.drift_acc = -ADJ_FREQ_MAX;
    target_trim = offset_ns / PTP_FREQ_P_DIV_SW + state->offset_pi.drift_acc;
    if (target_trim > ADJ_FREQ_MAX) target_trim = ADJ_FREQ_MAX;
    if (target_trim < -ADJ_FREQ_MAX) target_trim = -ADJ_FREQ_MAX;
  } else {
    int64_t interval_ns = state->local_time_ns_prev && local_delta_ns > 0 ?
        local_delta_ns : (int64_t)CONFIG_ESP_PTP_SYNC_INTERVAL_MS * 1000000;
    target_trim = ptp_wired_pi_step(&state->offset_pi.wired_integral_q16,
                                   offset_ns, interval_ns);
    state->offset_pi.drift_acc = state->offset_pi.wired_integral_q16 / 65536;
  }
  int32_t trim_delta = s_use_sw_clock ? (int32_t)target_trim - state->freq_trim_ppb :
      ptp_trim_relative_delta(state->freq_trim_ppb, (int32_t)target_trim);

  if (s_use_sw_clock) {
    /* The software backend clamps the cumulative rate at +/- 1e8 ppb.
     * Apply the change once per accepted observation. */
    if (ptp_clock_sw_adjtime_rate((int32_t)trim_delta) == 0) {
      state->freq_trim_ppb = (int32_t)target_trim;
    }
  } else {
    struct timex tx = {
        .modes = ADJ_FREQUENCY,
        .freq = trim_delta,
    };
    if (clock_adjtime(PTPD_CLOCK_ID, &tx) == 0) {
      state->freq_trim_ppb = (int32_t)target_trim;
    } else {
      state->offset_pi.wired_integral_q16 = saved_integral;
      state->offset_pi.drift_acc = saved_integral / 65536;
    }
  }

  state->remote_time_ns_prev = remote_time_ns;
  state->local_time_ns_prev = local_time_ns;

  ptpdebug("remote_delta_ns %lli, local_delta_ns %lli, tick_diff %lli",
          remote_delta_ns, local_delta_ns, tick_diff);
  ptpdebug("offset_ns %lli, trim %li, delta %li, drift_acc %li", offset_ns,
          (long)target_trim, (long)trim_delta,
          (long)state->offset_pi.drift_acc);
  // Get the path delay only when clock is stable enough. If we were
  // in the process of speeding/slowing the local clock, we'd get an
  // incorrect delay measurement
  int64_t diff = llabs(offset_ns) - llabs(state->last_offset_ns);
  static int cnt = 0;
  bool within_stability;
  if (s_use_sw_clock) {
    /* Phase never settles on the gap-noisy wifi reference, so judge lock by
     * the rate. Watch the integrator (drift_acc), not freq_trim: the latter
     * carries the proportional term, which is pure gap noise (tens of ppm)
     * even when the rate is locked. Require a windowed plateau — flat across
     * several seconds — so a slow integrator wind (small per-sample steps
     * but still climbing) is not mistaken for a lock. After a step the ring
     * holds stale values and naturally reports unstable until it refills. */
    static int32_t drift_ring[PTP_FREQ_STABLE_WIN];
    static int drift_ring_count = 0;
    static int drift_ring_pos = 0;
    drift_ring[drift_ring_pos] = state->offset_pi.drift_acc;
    drift_ring_pos = (drift_ring_pos + 1) % PTP_FREQ_STABLE_WIN;
    if (drift_ring_count < PTP_FREQ_STABLE_WIN) {
      drift_ring_count++;
    }
    int32_t dmin = drift_ring[0], dmax = drift_ring[0];
    for (int k = 1; k < drift_ring_count; k++) {
      if (drift_ring[k] < dmin) dmin = drift_ring[k];
      if (drift_ring[k] > dmax) dmax = drift_ring[k];
    }
    within_stability = (drift_ring_count >= PTP_FREQ_STABLE_WIN) &&
                       (dmax - dmin < PTP_FREQ_STABLE_PPB);
  } else {
    within_stability =
        (ptp_is_gptp(state) &&
         llabs(diff) < CONFIG_ESP_PTP_PEER_DELAY_STABILITY_NS) ||
        (!ptp_is_gptp(state) &&
         llabs(diff) < CONFIG_ESP_PTP_PATH_DELAY_STABILITY_NS);
  }
  if (within_stability) {
    if (cnt <= 3)
      cnt++;
  } else {
    cnt = 0;
  }
  if (cnt > 3) {
    ptpdebug("clock is stabilized");
    state->port[0].can_send_delayreq = true;
  } else {
    ptpdebug("clock is still unstable");
  }
  state->last_offset_ns = offset_ns;
}

static void ptp_clean_after_step(FAR struct ptp_state_s *state) {
  portENTER_CRITICAL(&s_peer_lock);
  ptp_peer_invalidate(&state->port[0].peer_exchange);
  portEXIT_CRITICAL(&s_peer_lock);
  ptp_invalidate_timing();
  state->remote_time_ns_prev = 0;
  state->local_time_ns_prev = 0;

  state->offset_pi.drift_acc = 0;
  state->offset_pi.wired_integral_q16 = (int64_t)state->freq_trim_ppb * 65536;
  state->last_offset_ns = 0;
}

/* Update local clock either by smooth adjustment or by jumping.
 * Remote time was remote_timestamp at local_timestamp.
 */

static int ptp_update_local_clock(FAR struct ptp_state_s *state,
                                  FAR struct timespec *remote_timestamp,
                                  FAR struct timespec *local_timestamp) {
  int ret = OK;
  int64_t delta_ns;
  int64_t absdelta_ns;
  const int64_t adj_limit_ns =
      CONFIG_ESP_PTP_SETTIME_THRESHOLD_MS * (int64_t)NSEC_PER_MSEC;

  ptpdebug("Local time: %lld.%09ld, remote time %lld.%09ld",
          (long long)local_timestamp->tv_sec, (long)local_timestamp->tv_nsec,
          (long long)remote_timestamp->tv_sec, (long)remote_timestamp->tv_nsec);

  delta_ns = timespec_delta_ns(remote_timestamp, local_timestamp);
  if (ptp_is_gptp(state)) {
    delta_ns += state->port[0].peer_delay_ns + state->correction_ns;
  } else {
    delta_ns += state->port[0].path_delay_ns + state->delay_asymmetry_ns;
  }
  absdelta_ns = (delta_ns < 0) ? -delta_ns : delta_ns;

  if (absdelta_ns > adj_limit_ns) {
    /* Large difference, move by jumping.
     * Account for delay since packet was received.
     */

    struct timespec new_time;
    ptp_gettime(state, &new_time);
    clock_timespec_subtract(&new_time, local_timestamp, &new_time);
    clock_timespec_add(&new_time, remote_timestamp, &new_time);
    ret = ptp_settime(state, &new_time);

    /* Reinitialize drift adjustment parameters */

    state->last_delta_timestamp = new_time;
    state->last_delta_ns = 0;
    state->last_adjtime_ns = 0;
    state->drift_avg_total_ms = 0;
    state->drift_ppb = 0;

    ptp_clean_after_step(state);

    if (ret == OK) {
      ptpinfo("Jumped to timestamp %lld.%09ld s\n", (long long)new_time.tv_sec,
              (long)new_time.tv_nsec);
    } else {
      ptperr("ptp_settime() failed: %d\n", errno);
    }
  } else {
    ptp_lock_local_clock_freq(state, remote_timestamp, local_timestamp);
  }

  return ret;
}

/* Process received PTP sync packet */

static int ptp_process_sync(FAR struct ptp_state_s *state,
                            FAR struct ptp_sync_s *msg) {
  struct timespec remote_time;

  if (memcmp(msg->header.sourceidentity,
             state->selected_source.header.sourceidentity,
             sizeof(msg->header.sourceidentity)) != 0 ||
      memcmp(msg->header.sourceportindex,
             state->selected_source.header.sourceportindex, 2) != 0) {
    /* This packet wasn't from the currently selected source */
    ESP_LOGD(TAG, "This packet wasn't from the currently selected source");
    return OK;
  }

  /* Update timeout tracking */

  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_sync);
  state->port[0].twostep_pending = false;

  if (msg->header.flags[0] & PTP_FLAGS0_TWOSTEP) {
    /* We need to wait for a follow-up packet before setting the clock. */

    state->port[0].twostep_rxtime = state->port[0].rxtime;
    state->port[0].twostep_packet = *msg;
    state->port[0].twostep_received_us = esp_timer_get_time();
    state->port[0].twostep_pending = true;
    ptpdebug("Waiting for follow-up");
    return OK;
  }

  /* Update local clock */

  ptp_format_to_timespec(msg->origintimestamp, &remote_time);
  return ptp_update_local_clock(state, &remote_time, &state->port[0].rxtime);
}

static int ptp_process_followup(FAR struct ptp_state_s *state,
                                FAR struct ptp_follow_up_s *msg) {
  struct timespec remote_time;

  int64_t age_us = esp_timer_get_time() - state->port[0].twostep_received_us;
  if (!state->port[0].twostep_pending || age_us < 0 || age_us > 1000000) return OK;

  if (memcmp(msg->header.sourceidentity,
             state->port[0].twostep_packet.header.sourceidentity,
             sizeof(msg->header.sourceidentity)) != 0 ||
      memcmp(msg->header.sourceportindex,
             state->port[0].twostep_packet.header.sourceportindex, 2) != 0) {
    return OK; /* This packet wasn't from the currently selected source */
  }

  if (ptp_get_sequence(&msg->header) !=
      ptp_get_sequence(&state->port[0].twostep_packet.header)) {
    ptpwarn("PTP follow-up packet sequence %ld does not match initial "
            "sync packet sequence %ld, ignoring\n",
            (long)ptp_get_sequence(&msg->header),
            (long)ptp_get_sequence(&state->port[0].twostep_packet.header));
    return OK;
  }

  if (ptp_is_gptp(state) &&
      (!ptp_timing_payload_valid((const uint8_t *)msg, sizeof(*msg)) ||
       !ptp_sync_receipt_interval((int8_t)state->port[0].twostep_packet.header.logmessageinterval)))
    return OK;

  /* Update local clock based on the remote timestamp we received now
   * and the local timestamp of when the sync packet was received.
   * For gPTP, we can also examine the information TLV for other changes
   */

  ptp_format_to_timespec(msg->origintimestamp, &remote_time);
  state->port[0].twostep_pending = false;
  if (ptp_is_gptp(state)) {
    state->correction_ns = get_correction_ns(msg->header.correction);
  }
  ptp_timing_snapshot_t previous;
  ptpd_timing_snapshot(&previous);
  int result = ptp_update_local_clock(state, &remote_time,
                                      &state->port[0].twostep_rxtime);
  ptp_timing_snapshot_t current;
  ptpd_timing_snapshot(&current);
  if (result == OK && ptp_is_gptp(state) &&
      ptp_timing_payload_valid((const uint8_t *)msg, sizeof(*msg))) {
    int64_t received_us = esp_timer_get_time();
    ptp_sync_receipt_observe(&state->port[0].sync_receipt,
        received_us, (int8_t)state->port[0].twostep_packet.header.logmessageinterval,
        received_us);
  }
  if (result == OK && current.generation == previous.generation &&
      ptp_is_gptp(state) && state->selected_source_valid &&
      ptp_timing_payload_valid((const uint8_t *)msg, sizeof(*msg))) {
    ptp_timing_snapshot_t next = {
      .generation = current.generation ? current.generation : 1,
      .received_us = esp_timer_get_time(),
      .reference_ns = timespec_to_ns(&remote_time),
      .local_receive_ns = timespec_to_ns(&state->port[0].twostep_rxtime),
      .correction_ns = state->correction_ns,
      .peer_delay_ns = state->port[0].peer_delay_ns,
      .trim_ppb = state->freq_trim_ppb,
      .drift_ppb = state->offset_pi.drift_acc,
      .hardware_clock = !s_use_sw_clock,
      .valid = true,
    };
    next.offset_ns = next.reference_ns + next.correction_ns + next.peer_delay_ns - next.local_receive_ns;
    memcpy(next.btc_identity, state->selected_source.btc_identity, 8);
    memcpy(next.own_identity, state->own_identity.header.sourceidentity, 8);
    memcpy(next.follow_up, msg, sizeof(next.follow_up));
    if (current.received_us && (memcmp(current.btc_identity, next.btc_identity, 8) ||
        memcmp(current.follow_up + 58, next.follow_up + 58, 2))) ++next.generation;
    portENTER_CRITICAL(&s_timing_lock);
    if (s_timing_snapshot.generation == current.generation) s_timing_snapshot = next;
    portEXIT_CRITICAL(&s_timing_lock);
  }
  return result;
}

static int ptp_process_delay_req(FAR struct ptp_state_s *state,
                                 FAR struct ptp_delay_req_s *req) {
  ptp_msgbuf
      resp; // using generic message buffer to allow for larger follow-up size
  struct timespec ts;
  int ret;

  if (!ptp_is_gptp(state) && state->selected_source_valid) {
    /* We are operating as a standard PTP client, ignore delay requests */

    return OK;
  }

  memset(&resp, 0, sizeof(resp));
  resp.header = state->own_identity.header;
  resp.header.messagetype =
      ptp_is_gptp(state) ? PTP_MSGTYPE_PDELAY_RESP : PTP_MSGTYPE_DELAY_RESP;
  size_t resp_len = sizeof(struct ptp_delay_resp_s);

  /* twoStepFlag belongs to Pdelay_Resp, not Delay_Resp (IEEE 1588-2019
   * Table 37); a Pdelay_Resp_Follow_Up always follows. */
  if (ptp_is_gptp(state)) {
    resp.header.flags[0] = PTP_FLAGS0_TWOSTEP;
    resp.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
    resp.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;   // gPTP required flag
    resp.header.controlfield = 5;
  }

  timespec_to_ptp_format(&state->port[0].rxtime,
                         resp.delay_resp.receivetimestamp);
  memcpy(resp.delay_resp.reqidentity, req->header.sourceidentity,
         sizeof(resp.delay_resp.reqidentity));
  memcpy(resp.delay_resp.reqportindex, req->header.sourceportindex,
         sizeof(resp.delay_resp.reqportindex));
  memcpy(resp.header.sequenceid, req->header.sequenceid,
         sizeof(resp.header.sequenceid));
  /* gPTP requires the 0x7F "no change" sentinel here per IEEE
   * 1588-2008 §13.6.2.2 and 802.1AS-2011 §11.4.5.4 / §11.4.5.5;
   * neither message dictates the requester's Pdelay_Req cadence.
   * Pdelay_Resp_Follow_Up inherits the value from this header. */
  resp.header.logmessageinterval = msec_to_log_period(
      ptp_is_gptp(state) ? log_period_to_msec(0x7F)
                         : CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS);
  if (ptp_is_gptp(state) && ptp_sends_endpoint_decl(state)) {
    resp_len += ptp_append_endpoint_decl_tlv(resp.raw, resp_len);
  }
  ptp_set_message_length(&resp.header, resp_len);

  /* A unicast Delay_Req gets a unicast Delay_Resp (IEEE 1588-2019 16.9,
   * a shall for AVB Lite timetransmitters). */
  bool unicast_reply = !ptp_is_gptp(state) &&
                       state->port[0].rx_dest_mac_valid &&
                       !(state->port[0].rx_dest_mac[0] & 0x01) &&
                       state->port[0].rx_source_mac_valid;

  /* Send the response message */

  if (unicast_reply) {
    resp.header.flags[0] |= PTP_FLAGS0_UNICAST;
    ret = ptp_net_send_to(state, &resp, resp_len, &ts,
                          state->port[0].rx_source_mac);
  } else {
    ret = ptp_net_send(state, &resp, resp_len, &ts);
  }

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
    return ret;
  }

  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_transmitted_delayresp);

  /* gPTP profile requires response follow-up message */

  if (ptp_is_gptp(state)) {
    /* Rebuild the buffer for follow-up: the prior TLV append was at the end
     * of the Pdelay_Resp body, but the follow-up has a different body size.
     * Clear the Pdelay_Resp TLV area before laying down the follow-up's TLV. */
    memset(resp.raw + sizeof(struct ptp_delay_resp_s), 0,
           sizeof(resp.raw) - sizeof(struct ptp_delay_resp_s));
    timespec_to_ptp_format(&ts, resp.delay_resp_follow_up.origintimestamp);
    resp.header.messagetype = PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP;
    resp.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
    size_t fup_len = sizeof(struct ptp_delay_resp_follow_up_s);
    if (ptp_sends_endpoint_decl(state)) {
      fup_len += ptp_append_endpoint_decl_tlv(resp.raw, fup_len);
    }
    ptp_set_message_length(&resp.header, fup_len);
    /* Clear PTP_TWO_STEP — nothing follows the Pdelay_Resp_Follow_Up
     * (IEEE 1588-2008 §13.3.2.6). */
    resp.header.flags[0] = 0;

    /* Send the response follow-up message, currently only for ESP as it
     * requires hw timestamp data */

    ret = ptp_net_send(state, &resp, fup_len, NULL);

    if (ret < 0) {
      ptperr("sendto for delay response follow-up message failed: %d\n", errno);
      return ret;
    }
    ptpdebug("Sent response + response follow-up, seq %ld",
            (long)ptp_get_sequence(&resp.header));
    /* asCapable-trigger diagnostic: per Pdelay response, log the inputs the
     * upstream peer uses to keep us asCapable — responder turnaround (t3-t2),
     * our applied clock-rate trim (its neighborRateRatio input), peer delay,
     * and time since the last Sync we received (gap = peer withholding). */
    {
      int64_t turn_us =
          (timespec_to_ns(&ts) - timespec_to_ns(&state->port[0].rxtime)) / 1000;
      struct timespec now_mono, sync_gap;
      clock_gettime(CLOCK_MONOTONIC, &now_mono);
      clock_timespec_subtract(&now_mono, &state->port[0].last_received_sync,
                              &sync_gap);
      ESP_LOGD("ptpd-trig",
               "turn_us=%lld freq_ppb=%ld pdelay_ns=%lld sync_gap_ms=%lld",
               (long long)turn_us, (long)state->freq_trim_ppb,
               (long long)state->port[0].peer_delay_ns,
               (long long)timespec_to_ms(&sync_gap));
    }
  } else {
    ptpdebug("Sent delay resp, seq %ld", (long)ptp_get_sequence(&req->header));
  }

  return OK;
}

static int ptp_process_delay_resp(FAR struct ptp_state_s *state,
                                  FAR struct ptp_delay_resp_s *msg) {
  int64_t path_delay;
  int64_t sync_delay;
  struct timespec remote_rxtime;
  uint16_t sequence;

  if (ptp_is_gptp(state)) {
    portENTER_CRITICAL(&s_peer_lock);
    int admission = ptp_peer_response(&state->port[0].peer_exchange, msg,
                                      &state->port[0].rxtime, esp_timer_get_time());
    if (state->port[0].peer_exchange.multiple)
      state->port[0].peer_capability.qualified = false;
    portEXIT_CRITICAL(&s_peer_lock);
    if (admission == PTP_PEER_MULTIPLE) {
      state->port[0].pdelay_multi_responder = true;
      return OK;
    }
    if (admission != PTP_PEER_ACCEPTED) return OK;
    ptpdebug("Waiting for delay response follow-up");
  } else {
    if (!state->selected_source_valid ||
        memcmp(msg->header.sourceidentity,
               state->selected_source.header.sourceidentity, 8) ||
        memcmp(msg->reqidentity, state->own_identity.header.sourceidentity, 8) ||
        memcmp(msg->reqportindex, state->own_identity.header.sourceportindex, 2))
      return OK;
    sequence = ptp_get_sequence(&msg->header);
    if (sequence != state->delay_req_seq) return OK;
    state->unicast_delay_req_outstanding = false;
    state->unicast_delay_req_misses = 0;
    /* Path delay is calculated as the average between delta for sync
     * message and delta for delay req message.
     * (IEEE-1588 section 11.3: Delay request-response mechanism)
     */

    ptp_format_to_timespec(msg->receivetimestamp, &remote_rxtime);
    path_delay =
        timespec_delta_ns(&remote_rxtime, &state->port[0].delayreq_time);
    /* Locked, t2 - t1 equals the path delay plus delayAsymmetry. */
    sync_delay = state->port[0].path_delay_ns + state->delay_asymmetry_ns -
                 state->last_delta_ns;
    path_delay = (path_delay + sync_delay) / 2;

    if (path_delay >= 0 &&
        path_delay < CONFIG_ESP_PTP_MAX_PATH_DELAY_NS) {
      if (state->port[0].path_delay_avgcount <
          CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT) {
        state->port[0].path_delay_avgcount++;
      }

      state->port[0].path_delay_ns +=
          (path_delay - state->port[0].path_delay_ns) /
          state->port[0].path_delay_avgcount;

      ptpinfo("Path delay: %ld ns (avg: %ld ns)\n", (long)path_delay,
              (long)state->port[0].path_delay_ns);
    } else {
      ptpwarn("Path delay out of range: %lld ns\n", (long long)path_delay);
    }
  }

  /* logmessageinterval is signed int8, log2(seconds). Valid per IEEE
   * 1588-2008 Annex F.4 is -8..+8. The 0x7F sentinel ("no change") and
   * any other out-of-range value leave the per-profile default
   * unchanged (set by ptp_initialize_state from PDELAYREQ/DELAYREQ
   * Kconfig). */
  if (!ptp_is_gptp(state)) {
    int8_t logmsg = (int8_t)msg->header.logmessageinterval;
    if (logmsg >= -8 && logmsg <= 8) {
      state->port[0].delayreq_interval_ms = log_period_to_msec(logmsg);
    }
  }

  /* Delay for next interval */
  state->port[0].next_delayreq_interval_ms =
      ptp_is_gptp(state)
          ? state->port[0].delayreq_interval_ms
          : rand_delayreq_interval(state->port[0].delayreq_interval_ms);
  ptpdebug("Randomized delay req interval: %d ms",
          state->port[0].next_delayreq_interval_ms);

  return OK;
}

static int
ptp_process_delay_resp_follow_up(FAR struct ptp_state_s *state,
                                 FAR struct ptp_delay_resp_follow_up_s *msg,
                                 uint32_t expected_generation) {
  if (!ptp_is_gptp(state)) {
    return OK;
  }

  int64_t peer_delay_roundtrip;
  int64_t peer_delay_reflection;
  double peer_delay = 0;
  ptp_peer_rate_t rate;
  struct timespec remote_txtime;
  struct timespec remote_rxtime;

  ptp_peer_measurement_t measurement;
  portENTER_CRITICAL(&s_peer_lock);
  if (expected_generation &&
      state->port[0].peer_exchange.generation != expected_generation) {
    portEXIT_CRITICAL(&s_peer_lock);
    return OK;
  }
  bool accepted = ptp_peer_finish(&state->port[0].peer_exchange, msg,
                                 esp_timer_get_time(), &measurement);
  int64_t early_us = state->port[0].peer_exchange.published_us -
                     state->port[0].peer_exchange.first_response_us;
  rate = state->port[0].peer_rate;
  if (state->port[0].peer_exchange.multiple)
    state->port[0].peer_capability.qualified = false;
  portEXIT_CRITICAL(&s_peer_lock);
  if (!accepted) return OK;

  peer_delay_roundtrip = timespec_delta_ns(&measurement.receive,
                                          &measurement.transmit);
  ptp_format_to_timespec(measurement.response.receivetimestamp, &remote_rxtime);
  ptp_format_to_timespec(msg->origintimestamp, &remote_txtime);
  if (remote_txtime.tv_sec < remote_rxtime.tv_sec ||
      remote_txtime.tv_sec - remote_rxtime.tv_sec > 1) return OK;
  peer_delay_reflection = timespec_delta_ns(&remote_txtime, &remote_rxtime);
  if (peer_delay_roundtrip < 0 || peer_delay_roundtrip > 1000000000LL ||
      peer_delay_reflection < 0 || peer_delay_reflection > 1000000000LL)
    return OK;
  const double response_correction = ptp_peer_correction(
      measurement.response.header.correction);
  const double follow_up_correction = ptp_peer_correction(msg->header.correction);
  uint8_t responder[10];
  memcpy(responder, msg->header.sourceidentity, 8);
  memcpy(responder + 8, msg->header.sourceportindex, 2);
  bool rate_valid = ptp_peer_rate_update(&rate, measurement.lifecycle, responder,
      &remote_txtime, follow_up_correction, &measurement.receive);
  bool delay_valid = rate_valid && ptp_peer_delay_local(rate.ratio,
      peer_delay_roundtrip, peer_delay_reflection, response_correction,
      follow_up_correction, &peer_delay) &&
      peer_delay < CONFIG_ESP_PTP_MAX_PEER_DELAY_NS;

  long rounded_delay = delay_valid ? (long)llround(peer_delay) : 0;
  portENTER_CRITICAL(&s_peer_lock);
  bool current = state->port[0].peer_exchange.generation == measurement.generation &&
      state->port[0].peer_exchange.lifecycle == measurement.lifecycle &&
      !state->port[0].peer_exchange.multiple;
  if (current) {
    if (state->port[0].peer_rate.lifecycle != rate.lifecycle ||
        memcmp(state->port[0].peer_rate.responder, rate.responder, 10)) {
      state->port[0].peer_delay_avgcount = 0;
      state->port[0].peer_delay_ns = 0;
    }
    state->port[0].peer_rate = rate;
    if (delay_valid) {
      if (state->port[0].peer_delay_avgcount < CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT)
        state->port[0].peer_delay_avgcount++;
      state->port[0].peer_delay_ns +=
          (rounded_delay - state->port[0].peer_delay_ns) /
          state->port[0].peer_delay_avgcount;
    }
    state->port[0].peer_capability = (ptp_peer_capability_t){
      .lifecycle = measurement.lifecycle,
      .received_us = esp_timer_get_time(),
      .qualified = delay_valid && state->port[0].peer_delay_ns <= 800 &&
          measurement.response.header.messagetype == 0x13 &&
          measurement.response.header.reserved1 == 0 &&
          msg->header.messagetype == 0x1a && msg->header.reserved1 == 0 &&
          memcmp(responder, state->own_identity.header.sourceidentity, 8) != 0,
    };
  }
  portEXIT_CRITICAL(&s_peer_lock);
  static unsigned early_completions;
  if (current && early_us > 0 && ((early_completions++ % 16) == 0))
    ptpinfo("PDELAY_EARLY,%u,%lld", early_completions, (long long)early_us);
  if (current && rate_valid && (rate.updates == 1 || !(rate.updates % 16)))
    ptpinfo("PDELAY_RATE,%u,%ld,%ld,%d", rate.updates,
            (long)llround((rate.ratio - 1.0) * 1e9),
            (long)llround(peer_delay), delay_valid);

  return OK;
}

/* Determine received packet type and process it */

static int ptp_process_rx_packet(FAR struct ptp_state_s *state,
                                 ssize_t length, int ingress_port) {
  if (ingress_port < 0 || ingress_port >= CONFIG_ESP_PTP_NUM_PORTS) return -EINVAL;
  if (length < sizeof(struct ptp_header_s)) {
    ptpwarn("Ignoring invalid PTP packet, length only %d bytes\n", (int)length);
    return OK;
  }

  size_t bounded_length = ptp_message_bounded_length(state->port[0].rxbuf.raw,
                                                     (size_t)length);
  if (!bounded_length) return OK;
  length = bounded_length;
  if (ptp_is_gptp(state) &&
      (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK) == PTP_MSGTYPE_FOLLOW_UP &&
      !ptp_timing_payload_valid(state->port[0].rxbuf.raw, bounded_length)) return OK;

  if (state->port[0].rxbuf.header.domain != CONFIG_ESP_PTP_DOMAIN) {
    /* Part of different clock domain, ignore. Hexdump the first
     * rejects and then a decimated sample: after an EMAC RX stall the
     * driver has been observed to deliver correctly-SIZED frames whose
     * CONTENT fails this check forever — the dump shows what actually
     * arrives (stale/desynced buffer suspect, P4 RX recovery path). */
    ptpd_reject_dump("domain", state->port[0].rxbuf.raw, length);
    return OK;
  }

  bool msg_is_gptp =
      (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_SDOID_GPTP) != 0;
  if (msg_is_gptp != ptp_is_gptp(state)) {
    /* Frame passed the domain check, so content is sane — the socket
     * is alive even though policy rejects the frame. */
    clock_gettime(CLOCK_MONOTONIC, &state->port[ingress_port].last_socket_alive);
    /* §2.3 endpoint beacons are deliberately SDOID-tagged for gPTP
     * receivers; a fallen-back peer sees them here at steady state.
     * Expected traffic — keep it out of the capped reject log. */
    if ((state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK) ==
        PTP_MSGTYPE_PDELAY_REQ) {
      ptpdebug("beacon pdelay_req ignored (sdoid), len=%d", (int)length);
    } else {
      ptpd_reject_dump("sdoid", state->port[0].rxbuf.raw, length);
    }
    return OK;
  }

  /* STA timing belongs to the currently associated AP. */
  struct ptp_port_s *ingress = &state->port[ingress_port];
  if (ptp_is_gptp(state) && ingress->medium == ptp_port_medium_wifi_ftm &&
      ingress->wifi_mode == ptp_port_wifi_mode_sta) {
    portENTER_CRITICAL(&s_peer_lock);
    bool associated_sender = state->port[0].rx_source_mac_valid &&
        ptp_wifi_neighbor_accepts(&ingress->wifi_neighbor, state->port[0].rx_source_mac);
    state->port[0].rx_association = ingress->wifi_neighbor.association;
    portEXIT_CRITICAL(&s_peer_lock);
    if (!associated_sender) return OK;
  }

  if (ptp_is_gptp(state) && ingress->medium == ptp_port_medium_wifi_ftm &&
      ingress->wifi_mode == ptp_port_wifi_mode_ap) {
    portENTER_CRITICAL(&s_peer_lock);
    ptp_wifi_peer_t *peer = state->port[0].rx_source_mac_valid ?
        ptp_wifi_peers_find(&ingress->wifi_peers, state->port[0].rx_source_mac) : NULL;
    bool associated_sender = peer != NULL && ptp_wifi_peers_publishable(&ingress->wifi_peers);
    state->port[0].rx_association = peer ? peer->association : 0;
    portEXIT_CRITICAL(&s_peer_lock);
    if (!associated_sender) return OK;
  }

  clock_gettime(CLOCK_MONOTONIC, &state->port[ingress_port].last_received_multicast);

  /* Timing state currently belongs to the bootstrap port only. */
  uint8_t timing_type = state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK;
#ifdef CONFIG_ESP_PTP_SOURCE_LOSS_PROBE
  if (ingress_port == 0 && ptp_is_gptp(state) &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      ptp_source_loss_probe_drop(&s_source_loss_probe, timing_type)) return OK;
#endif
  if (timing_type == PTP_MSGTYPE_ANNOUNCE || timing_type == PTP_MSGTYPE_SYNC ||
      timing_type == PTP_MSGTYPE_FOLLOW_UP) {
    if (ingress_port != 0) return OK;
    if (ptp_is_gptp(state) && state->port[0].medium == ptp_port_medium_eth_hwts &&
        !ptp_wired_capability(state)) {
      state->port[0].twostep_pending = false;
      return OK;
    }
  }

  /* Rout the packet to the appropriate handler */

  switch (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK) {
  case PTP_MSGTYPE_SIGNALING: {
    uint8_t local_port[10];
    memcpy(local_port, state->own_identity.header.sourceidentity, 8);
    local_port[8] = (uint8_t)((ingress_port + 1) >> 8);
    local_port[9] = (uint8_t)(ingress_port + 1);
    if (ingress->medium == ptp_port_medium_wifi_ftm &&
        ingress->wifi_mode == ptp_port_wifi_mode_ap) {
      portENTER_CRITICAL(&s_peer_lock);
      ptp_wifi_peer_t *peer = state->port[0].rx_source_mac_valid ?
          ptp_wifi_peers_find(&ingress->wifi_peers, state->port[0].rx_source_mac) : NULL;
      uint16_t logical_port = peer && peer->association == state->port[0].rx_association ?
          ptp_wifi_association_port(ingress_port + 1, CONFIG_ESP_PTP_NUM_PORTS,
              (unsigned)(peer - ingress->wifi_peers.entries)) : 0;
      portEXIT_CRITICAL(&s_peer_lock);
      if (!logical_port) return OK;
      local_port[8] = logical_port >> 8;
      local_port[9] = logical_port;
    }
    ptp_capable_message_t indication;
    int decoded = ptp_signaling_read_capable(state->port[0].rxbuf.raw, length,
        CONFIG_ESP_PTP_DOMAIN, local_port, &indication);
    ptp_capable_message_t interval_request;
    int interval_decoded = ptp_signaling_read_capable_interval(state->port[0].rxbuf.raw,
        length, CONFIG_ESP_PTP_DOMAIN, local_port, &interval_request);
    ptp_interval_message_t timing_request;
    int timing_decoded = ptp_signaling_read_message_interval(state->port[0].rxbuf.raw,
        length, CONFIG_ESP_PTP_DOMAIN, local_port, &timing_request);
    /* Validate all known TLVs before any can change association state. */
    if (decoded == PTP_SIGNALING_MALFORMED || interval_decoded == PTP_SIGNALING_MALFORMED ||
        timing_decoded == PTP_SIGNALING_MALFORMED)
      return OK;
    if (decoded == PTP_SIGNALING_CAPABLE) {
      struct ptp_port_s *port = &state->port[ingress_port];
      bool indication_bound = false;
      /* Bind indications to the measured wired peer or associated STA neighbor. */
      portENTER_CRITICAL(&s_peer_lock);
      if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_eth_hwts && port->peer_rate.valid &&
          port->peer_rate.lifecycle == port->peer_exchange.lifecycle)
        indication_bound = ptp_capable_receive(&port->capable_receive, &indication,
            port->peer_rate.responder, port->peer_exchange.lifecycle,
            esp_timer_get_time());
      else if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_sta && port->wifi_neighbor.bound &&
          state->port[0].rx_association == port->wifi_neighbor.association &&
          state->port[0].rx_source_mac_valid &&
          ptp_wifi_neighbor_accepts(&port->wifi_neighbor, state->port[0].rx_source_mac))
        indication_bound = ptp_capable_receive(&port->capable_receive, &indication,
            port->wifi_neighbor.port_identity, port->wifi_neighbor.association,
            esp_timer_get_time());
      else if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_ap && state->port[0].rx_source_mac_valid)
        indication_bound = ptp_wifi_peers_capable(&port->wifi_peers, state->port[0].rx_source_mac,
            state->port[0].rx_association, &indication, esp_timer_get_time());
      portEXIT_CRITICAL(&s_peer_lock);
#if defined(CONFIG_ESP_PTP_INTERVAL_PROBE) || defined(CONFIG_ESP_PTP_CAPABLE_RX_TRACE)
      ptpinfo("CAPTEST_RX,%lld,%u,%d", esp_timer_get_time(),
          ((unsigned)state->port[0].rxbuf.raw[30] << 8) | state->port[0].rxbuf.raw[31],
          (int)indication.log_interval);
#endif
      port->last_capable_indication = indication;
      clock_gettime(CLOCK_MONOTONIC, &port->last_received_capable_indication);
      if ((port->capable_indication_count++ % 32) == 0)
        ptpinfo("Capability indication on port %d, log interval %d, bound %u, media verification pending",
                ingress_port, (int)indication.log_interval, indication_bound);
    }
    if (timing_decoded == PTP_SIGNALING_MESSAGE_INTERVAL) {
      struct ptp_port_s *port = &state->port[ingress_port];
      portENTER_CRITICAL(&s_peer_lock);
      if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_ap && state->port[0].rx_source_mac_valid) {
        ptp_wifi_peers_sync_interval(&port->wifi_peers, state->port[0].rx_source_mac,
            state->port[0].rx_association, &timing_request);
        ptp_wifi_peers_announce_interval(&port->wifi_peers, state->port[0].rx_source_mac,
            state->port[0].rx_association, &timing_request,
            msec_to_log_period(CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS));
      }
      else if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_sta && state->port[0].rx_source_mac_valid)
        ptp_wifi_neighbor_sync_interval(&port->wifi_neighbor, state->port[0].rx_source_mac,
            state->port[0].rx_association, &timing_request);
      portEXIT_CRITICAL(&s_peer_lock);
    }
    if (interval_decoded == PTP_SIGNALING_CAPABLE_INTERVAL) {
      struct ptp_port_s *port = &state->port[ingress_port];
      portENTER_CRITICAL(&s_peer_lock);
      if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_ap && state->port[0].rx_source_mac_valid)
        ptp_wifi_peers_interval(&port->wifi_peers, state->port[0].rx_source_mac,
            state->port[0].rx_association, &interval_request);
      else if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_wifi_ftm &&
          port->wifi_mode == ptp_port_wifi_mode_sta && state->port[0].rx_source_mac_valid)
        ptp_wifi_neighbor_interval(&port->wifi_neighbor, state->port[0].rx_source_mac,
            state->port[0].rx_association, &interval_request);
      else if (port->enabled && port->link_up && ptp_is_gptp(state) &&
          port->medium == ptp_port_medium_eth_hwts && port->peer_rate.valid &&
          port->peer_rate.lifecycle == port->peer_exchange.lifecycle) {
        ptp_wired_capable_bind(&port->wired_capable, port->peer_exchange.lifecycle,
                               port->peer_rate.responder);
        ptp_wired_capable_request(&port->wired_capable, port->peer_exchange.lifecycle,
                                  &interval_request);
      }
      portEXIT_CRITICAL(&s_peer_lock);
    }
    return OK;
  }

#if defined(CONFIG_ESP_PTP_CLIENT) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always acts as a client
  case PTP_MSGTYPE_ANNOUNCE:
    s_ptpd_rx_announce++;
    ptpdebug("Got announce packet, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    return ptp_process_announce(state, &state->port[0].rxbuf.announce, length);

  case PTP_MSGTYPE_SYNC:
    ptpd_lateness_record_rx_sync();
    ptpdebug("Got sync packet, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (!state->selected_source_valid ||
        (ptp_is_gptp(state) && state->selected_source.btc_priority1 == 255)) {
      return OK;
    } // ignore if operating as a server in gPTP profile
    return ptp_process_sync(state, &state->port[0].rxbuf.sync);

  case PTP_MSGTYPE_FOLLOW_UP:
    s_ptpd_rx_followup++;
    ptpdebug("Got follow-up packet, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (!state->selected_source_valid ||
        (ptp_is_gptp(state) && state->selected_source.btc_priority1 == 255)) {
      return OK;
    } // ignore if operating as a server in gPTP profile
    return ptp_process_followup(state, &state->port[0].rxbuf.follow_up);

  case PTP_MSGTYPE_DELAY_RESP:
  case PTP_MSGTYPE_PDELAY_RESP:
    if (ingress_port != 0) return OK;
    if (ptp_is_gptp(state) &&
        (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK) != PTP_MSGTYPE_PDELAY_RESP)
      return OK;
    s_ptpd_rx_pdelay_resp++;
    ptpdebug("Got delay-resp, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(state->port[0].rxbuf.raw, length,
                                      sizeof(struct ptp_delay_resp_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    return ptp_process_delay_resp(state, &state->port[0].rxbuf.delay_resp);
#endif

#if defined(CONFIG_ESP_PTP_SERVER) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always responds to delay
                                               // requests
  case PTP_MSGTYPE_DELAY_REQ:
  case PTP_MSGTYPE_PDELAY_REQ:
    s_ptpd_rx_pdelay_req++;
    ptpdebug("Got delay req, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(state->port[0].rxbuf.raw, length,
                                      sizeof(struct ptp_pdelay_req_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    return ptp_process_delay_req(state, &state->port[0].rxbuf.delay_req);
#endif

  case PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP:
    if (ingress_port != 0) return OK;
    s_ptpd_rx_pdelay_fup++;
    ptpdebug("Got peer delay resp follow-up, seq %ld",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(
            state->port[0].rxbuf.raw, length,
            sizeof(struct ptp_delay_resp_follow_up_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    return ptp_process_delay_resp_follow_up(
        state, &state->port[0].rxbuf.delay_resp_follow_up, 0);
  default:
    ptpinfo("Ignoring unknown PTP packet type: 0x%02x\n",
            state->port[0].rxbuf.header.messagetype);
    return OK;
  }
}

/* Signal handler for status / stop requests */

/* Process status information request */

/* 12.4 for the STA port: media support, a granted small burst and a fresh
 * gPTP-capable indication from the bound neighbor. Independent of the servo. */
static bool ptp_wifi_capability(FAR struct ptp_state_s *state, int index)
{
  struct ptp_port_s *port = &state->port[index];
  if (port->medium != ptp_port_medium_wifi_ftm ||
      port->wifi_mode != ptp_port_wifi_mode_sta || !port->enabled ||
      !port->link_up || !ptp_is_gptp(state)) return false;
  ptp_wifi_media_t media;
  if (!ptp_wifi_sta_media(index, &media)) return false;
  portENTER_CRITICAL(&s_peer_lock);
  bool neighbor = port->wifi_neighbor.associated && port->wifi_neighbor.bound &&
      ptp_neighbor_capable(&port->capable_receive, port->wifi_neighbor.port_identity,
                           port->wifi_neighbor.association, esp_timer_get_time());
  portEXIT_CRITICAL(&s_peer_lock);
  return ptp_wifi_as_capable(&media, neighbor, CONFIG_ESP_PTP_DOMAIN);
}

void ptp_wifi_sta_capability_status(int index, uint8_t port_identity[10],
                                    uint8_t *reason)
{
  memset(port_identity, 0, 10);
  *reason = 2;
  if (!s_state || index < 0 || index >= CONFIG_ESP_PTP_NUM_PORTS) return;
  struct ptp_port_s *port = &s_state->port[index];
  portENTER_CRITICAL(&s_peer_lock);
  if (port->wifi_neighbor.associated && port->wifi_neighbor.bound)
    memcpy(port_identity, port->wifi_neighbor.port_identity, 10);
  portEXIT_CRITICAL(&s_peer_lock);
  if (ptp_wifi_capability(s_state, index)) {
    *reason = 0;
    return;
  }
  /* The first of the 12.4 conditions that fails: media support, then the
   * FTM grant, then the neighbor's gPTP-capable Signaling. */
  ptp_wifi_media_t media;
  uint8_t support = ptp_wifi_sta_media(index, &media)
      ? ptp_wifi_tm_ftm_support(&media) : 0;
  if (!support) return;
  bool small_burst = media.granted_frames == 3 || media.granted_frames == 2;
  *reason = !(support & 1) && !small_burst ? 1 : 3;
}

static bool ptp_wired_capability(FAR struct ptp_state_s *state)
{
  portENTER_CRITICAL(&s_peer_lock);
  bool capable = state->port[0].medium == ptp_port_medium_eth_hwts &&
      ptp_peer_capable(&state->port[0].peer_capability,
          state->port[0].peer_exchange.lifecycle, esp_timer_get_time(),
          (int64_t)state->port[0].delayreq_interval_ms * 1000,
          state->port[0].enabled, state->port[0].link_up,
          ptp_is_gptp(state), CONFIG_ESP_PTP_DOMAIN,
          ptp_neighbor_capable(&state->port[0].capable_receive,
              state->port[0].peer_rate.responder,
              state->port[0].peer_exchange.lifecycle, esp_timer_get_time()));
  portEXIT_CRITICAL(&s_peer_lock);

  return capable;
}

static void ptp_process_statusreq(FAR struct ptp_state_s *state) {
  FAR struct ptpd_status_s *status;

  if (!state->status_req.dest) {
    return; /* No active request */
  }

  status = state->status_req.dest;
  status->ptp_profile = state->active_ptp_profile;
  status->peer_is_endpoint = state->port[0].peer_is_endpoint;
  status->avb_lite_fallback_reason = state->avb_lite_fallback_reason;
  status->domain = CONFIG_ESP_PTP_DOMAIN;
  status->link_speed_mbps = state->port[0].negotiated_link_mbps;
  status->gm_link_speed_mbps = state->selected_gm_link_mbps;
  status->delay_asymmetry_ns = state->delay_asymmetry_ns;
  status->clock_source_selected = state->selected_source_valid;
  memset(&status->selected_path, 0, sizeof(status->selected_path));
  if (status->clock_source_selected) status->selected_path = state->selected_path;
  status->clock_source_valid = status->clock_source_selected &&
      (!ptp_is_gptp(state) || state->selected_source.btc_priority1 < 255);
  if (status->clock_source_valid && s_use_sw_clock && ptp_ftm_clock_ready) {
    ptp_timing_snapshot_t timing;
    ptpd_timing_snapshot(&timing);
    status->clock_source_valid = ptp_ftm_clock_ready(timing.generation);
  }

  status->as_capable = state->port[0].medium == ptp_port_medium_wifi_ftm
      ? ptp_wifi_capability(state, 0) : ptp_wired_capability(state);

  /* Copy own identity info to status struct */

  FAR struct ptp_announce_s *o = &state->own_identity;

  memcpy(status->own_identity_info.id, o->header.sourceidentity,
         sizeof(status->own_identity_info.id));

  status->own_identity_info.utcoffset =
      (int16_t)(((uint16_t)o->utcoffset[0] << 8) | o->utcoffset[1]);
  status->own_identity_info.priority1 = o->btc_priority1;
  status->own_identity_info.clockclass = o->btc_quality[0];
  status->own_identity_info.accuracy = o->btc_quality[1];
  status->own_identity_info.priority2 = o->btc_priority2;
  status->own_identity_info.variance =
      ((uint16_t)o->btc_quality[2] << 8) | o->btc_quality[3];
  memcpy(status->own_identity_info.btc_id, o->btc_identity,
         sizeof(status->own_identity_info.btc_id));

  status->own_identity_info.stepsremoved =
      ((uint16_t)o->stepsremoved[0] << 8) | o->stepsremoved[1];
  status->own_identity_info.timesource = o->timesource;

  memset(&status->clock_source_info, 0, sizeof(status->clock_source_info));
  if (status->clock_source_selected) {
    /* Copy relevant parts of selected source announce info to status struct */

    FAR struct ptp_announce_s *s = &state->selected_source;

    memcpy(status->clock_source_info.id, s->header.sourceidentity,
           sizeof(status->clock_source_info.id));

    status->clock_source_info.utcoffset =
        (int16_t)(((uint16_t)s->utcoffset[0] << 8) | s->utcoffset[1]);
    status->clock_source_info.priority1 = s->btc_priority1;
    status->clock_source_info.clockclass = s->btc_quality[0];
    status->clock_source_info.accuracy = s->btc_quality[1];
    status->clock_source_info.priority2 = s->btc_priority2;
    status->clock_source_info.variance =
        ((uint16_t)s->btc_quality[2] << 8) | s->btc_quality[3];

    memcpy(status->clock_source_info.btc_id, s->btc_identity,
           sizeof(status->clock_source_info.btc_id));

    status->clock_source_info.stepsremoved =
        ((uint16_t)s->stepsremoved[0] << 8) | s->stepsremoved[1];
    status->clock_source_info.timesource = s->timesource;
  }

  /* Copy latest adjustment info */

  status->last_clock_update = state->last_delta_timestamp;
  status->last_delta_ns = state->last_offset_ns; /* servo offset from the BTC */
  status->last_adjtime_ns = state->last_adjtime_ns;
  status->drift_ppb = state->drift_ppb;
  status->path_delay_ns = state->port[0].path_delay_ns;
  status->peer_delay_ns = state->port[0].peer_delay_ns;

  /* Copy timestamps */

  status->last_received_multicast = state->port[0].last_received_multicast;
  status->last_received_announce = state->port[0].last_received_announce;
  status->last_received_sync = state->port[0].last_received_sync;
  status->last_transmitted_sync = state->port[0].last_transmitted_sync;
  status->last_transmitted_announce = state->port[0].last_transmitted_announce;
  status->last_transmitted_delayresp =
      state->port[0].last_transmitted_delayresp;
  status->last_transmitted_delayreq = state->port[0].last_transmitted_delayreq;

  /* Post semaphore to inform that we are done */

  if (state->status_req.done) {
    sem_post(state->status_req.done);
  }

  state->status_req.done = NULL;
  state->status_req.dest = NULL;
}

/* Main PTPD task */
static void ptp_daemon(void *task_param) {
  FAR struct ptp_state_s *state;
  struct pollfd pollfds[1]; // everything is received over one socket at L2
  struct ptp_bootstrap_args_s *args = (struct ptp_bootstrap_args_s *)task_param;
  int ret;

  state = calloc(1, sizeof(struct ptp_state_s));

  int init_rc = ptp_initialize_state(state, args);
  free(args); /* heap struct from ptpd_start / ptpd_start_port */
  if (init_rc != OK) {
    ptperr("Failed to initialize PTP state, exiting\n");

    ptp_destroy_state(state);
    free(state);

    goto err;
  }

  pollfds[0].events = POLLIN;
  pollfds[0].fd = state->port[0].ptp_socket;

/* Wi-Fi-medium ports have no L2TAP socket (ptp_socket = -1); sync
 * arrives asynchronously via ptpd_inject_sync from the beacon-IE
 * handler. poll() on fd<0 just waits the full timeout, so we cap it
 * to PTPD_NOSOCK_POLL_MS to keep periodic work — including
 * ptp_process_statusreq — responsive. With the default 10 s
 * interval, ptpd_status() (1 s timeout) would otherwise always time
 * out and AVB_INTERFACE / CLOCK_SOURCE descriptors never refresh. */
#define PTPD_NOSOCK_POLL_MS 100
/* Cap the RX poll so periodic TX (announce/sync, the 3 s §2.3 beacon,
 * Pdelay) keeps schedule on a quiet wire. On gPTP networks inbound
 * traffic wakes poll far more often than this, so the cap only bounds
 * the worst case: AVB-Lite through a non-AVB switch, where all TX
 * cadence used to collapse to the 10 s poll timeout. */
#define PTPD_POLL_CAP_MS 500
  int poll_timeout =
      (pollfds[0].fd < 0) ? PTPD_NOSOCK_POLL_MS
      : (PTPD_POLL_INTERVAL < PTPD_POLL_CAP_MS ? PTPD_POLL_INTERVAL
                                               : PTPD_POLL_CAP_MS);

  /* L2TAP RX-starvation watchdog (see ptp_port_reopen_l2tap). On a
   * wired gPTP port there is Pdelay traffic at least once a second in
   * any healthy AVB domain, so multi-second silence with the link up
   * means the fd has wedged. Keyed on last_received_multicast — the
   * CLOCK_MONOTONIC stamp taken only for VALID in-domain gPTP
   * messages in ptp_process_rx_packet — because a wedged fd has been
   * observed to keep waking poll() with reads that never parse as
   * in-domain PTP, which would defeat a read()-based liveness key. */
#define PTPD_RX_STARVATION_US 5000000LL
  int64_t watchdog_rearm_us = esp_timer_get_time();

  while (!state->stop) {
    ptpd_lateness_tick();
#ifdef CONFIG_ESP_PTP_SOURCE_LOSS_PROBE
    ptp_timing_snapshot_t probe_timing;
    bool probe_ready = state->port[0].medium == ptp_port_medium_eth_hwts &&
        ptp_is_gptp(state) && state->selected_source_valid &&
        ptpd_timing_snapshot(&probe_timing);
    if (ptp_source_loss_probe_tick(&s_source_loss_probe, esp_timer_get_time(), probe_ready))
      ptpinfo("SOURCELOSS,%u,%u,%u", s_source_loss_probe.phase,
          s_source_loss_probe.dropped[1], s_source_loss_probe.dropped[3]);
#endif
    if (state->port[0].medium == ptp_port_medium_eth_hwts) {
      bool capable = ptp_wired_capability(state);
      if (ptp_is_gptp(state) && !capable) {
        if (state->selected_source_valid || state->port[0].twostep_pending)
          ptp_invalidate_timing();
        state->selected_source_valid = false;
        state->port[0].twostep_pending = false;
      }
      if (!state->port[0].capability_reported ||
          capable != state->port[0].last_reported_capability) {
        ptpinfo("PDELAY_CAPABLE,%u", capable);
        state->port[0].capability_reported = true;
        state->port[0].last_reported_capability = capable;
      }
    } else if (state->port[0].medium == ptp_port_medium_wifi_ftm &&
               state->port[0].wifi_mode == ptp_port_wifi_mode_sta) {
      bool capable = ptp_wifi_capability(state, 0);
      if (!state->port[0].capability_reported ||
          capable != state->port[0].last_reported_capability) {
        ptpinfo("WIFI_CAPABLE,%u", capable);
        state->port[0].capability_reported = true;
        state->port[0].last_reported_capability = capable;
      }
    }
    state->port[0].can_send_delayreq = ptp_is_gptp(state);

    pollfds[0].fd = state->port[0].ptp_socket;
    pollfds[0].revents = 0;

    /* While acting as the BTC on a wired port, Sync must go out every
     * CONFIG_ESP_PTP_SYNC_INTERVAL_MS (125 ms, logMessageInterval -3)
     * in both profiles. TX happens once per loop pass, and on a quiet
     * wire poll() sleeps up to PTPD_POLL_CAP_MS between passes,
     * capping Sync at ~2/s, so peers' syncReceiptTimeout (3 intervals)
     * expires on every gap and they never lock to us. The standard
     * profile after an AVB-Lite fallback runs on a quiet non-AVB
     * switch, so it needs this cap as much as gPTP. */
#define PTPD_BTC_POLL_CAP_MS 25
    int wait_ms = poll_timeout;
    if (ptp_ftm_daemon_tick && state->port[0].medium == ptp_port_medium_wifi_ftm && wait_ms > 50)
      wait_ms = 50;
    if (!state->selected_source_valid &&
        state->port[0].medium == ptp_port_medium_eth_hwts &&
        wait_ms > PTPD_BTC_POLL_CAP_MS) {
      wait_ms = PTPD_BTC_POLL_CAP_MS;
    }
    portENTER_CRITICAL(&s_peer_lock);
    bool pending_peer_follow_up = state->port[0].peer_exchange.follow_up_pending &&
        !state->port[0].peer_exchange.multiple &&
        esp_timer_get_time() < state->port[0].peer_exchange.deadline_us;
    portEXIT_CRITICAL(&s_peer_lock);
    if (pending_peer_follow_up && wait_ms > 1) wait_ms = 1;
    /* Capability deadlines must not depend on incoming Ethernet traffic. */
    ptp_wifi_capable_completions(state);
    int64_t capable_now = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_lock);
    if (ptp_is_gptp(state) && state->port[0].medium == ptp_port_medium_eth_hwts &&
        state->port[0].enabled && state->port[0].link_up && state->port[0].ptp_socket >= 0)
      wait_ms = ptp_capable_schedule_wait_ms(&state->port[0].wired_capable.transmit,
                                            capable_now, wait_ms);
    for (int index = 0; index < CONFIG_ESP_PTP_NUM_PORTS; ++index) {
      struct ptp_port_s *port = &state->port[index];
      if (!ptp_is_gptp(state) || !port->enabled || !port->link_up ||
          port->medium != ptp_port_medium_wifi_ftm) continue;
      if (port->wifi_mode == ptp_port_wifi_mode_sta) {
        if (port->wifi_neighbor.associated)
          wait_ms = ptp_capable_schedule_wait_ms(&port->wifi_neighbor.transmit, capable_now, wait_ms);
        continue;
      }
      if (port->wifi_mode != ptp_port_wifi_mode_ap ||
          !ptp_wifi_peers_publishable(&port->wifi_peers)) continue;
      for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
        ptp_wifi_peer_t *peer = &port->wifi_peers.entries[slot];
        if (peer->associated)
          wait_ms = ptp_capable_schedule_wait_ms(&peer->transmit, capable_now, wait_ms);
      }
    }
    portEXIT_CRITICAL(&s_peer_lock);
    if (ptp_is_gptp(state) && state->selected_source_valid &&
        state->selected_source.btc_priority1 < 255)
      wait_ms = ptp_sync_receipt_wait_ms(&state->port[0].sync_receipt,
                                        esp_timer_get_time(), wait_ms);
    ret = poll(pollfds, 1, wait_ms);

    if (pollfds[0].revents) {
      /* Receive time-critical packet, potentially with cmsg
       * indicating the timestamp.
       */
      s_ptpd_poll_wake++;

      ret = ptp_net_recv(state, &state->port[0].rxbuf,
                         sizeof(state->port[0].rxbuf), &state->port[0].rxtime);

      if (ret > 0) {
        ptp_process_rx_packet(state, ret, 0);
      }
    }

    /* Bound each drain so a busy producer cannot starve periodic work. */
    for (unsigned drained = 0; drained < PTP_INJECT_DEPTH; ++drained) {
      struct ptp_injected_frame_s incoming;
      portENTER_CRITICAL(&s_injected_lock);
      bool available = s_injected_count != 0;
      if (available) {
        incoming = s_injected[s_injected_head];
        s_injected_head = (s_injected_head + 1) % PTP_INJECT_DEPTH;
        --s_injected_count;
      }
      bool current = available && incoming.link_generation ==
          s_injected_generation[incoming.port_index];
      portEXIT_CRITICAL(&s_injected_lock);
      if (!available) break;
      if (!current) continue;
      state->port[0].rxbuf = incoming.message;
      state->port[0].rxtime = incoming.received;
      memcpy(state->port[0].rx_source_mac, incoming.source_mac, 6);
      state->port[0].rx_source_mac_valid = incoming.source_mac_valid;
      state->port[0].rx_dest_mac_valid = false;
      ptp_process_rx_packet(state, incoming.length, incoming.port_index);
    }

    struct ptp_delay_resp_follow_up_s deferred_follow_up;
    uint32_t deferred_generation;
    portENTER_CRITICAL(&s_peer_lock);
    bool deferred = ptp_peer_take_follow_up(&state->port[0].peer_exchange,
        esp_timer_get_time(), &deferred_follow_up, &deferred_generation);
    portEXIT_CRITICAL(&s_peer_lock);
    if (deferred)
      ptp_process_delay_resp_follow_up(state, &deferred_follow_up, deferred_generation);

    /* Starvation check runs UNCONDITIONALLY every loop iteration —
     * gating it on the poll outcome or on read() success would mask
     * the wedge (see comment at PTPD_RX_STARVATION_US). Silence is
     * normal for a standard-profile timetransmitter with no
     * timereceivers on the segment, so there it only runs while a
     * source is selected (Sync then arrives 8 times a second). */
    if (state->port[0].medium == ptp_port_medium_eth_hwts &&
        (ptp_is_gptp(state) || state->selected_source_valid)) {
      int64_t now_us = esp_timer_get_time();
      struct timespec mono;
      clock_gettime(CLOCK_MONOTONIC, &mono);
      int64_t mono_us =
          (int64_t)mono.tv_sec * 1000000LL + mono.tv_nsec / 1000LL;
      int64_t last_valid_us =
          (int64_t)state->port[0].last_received_multicast.tv_sec * 1000000LL +
          state->port[0].last_received_multicast.tv_nsec / 1000LL;
      int64_t last_alive_us =
          (int64_t)state->port[0].last_socket_alive.tv_sec * 1000000LL +
          state->port[0].last_socket_alive.tv_nsec / 1000LL;
      if (last_alive_us > last_valid_us)
        last_valid_us = last_alive_us;
      if (mono_us - last_valid_us > PTPD_RX_STARVATION_US &&
          now_us - watchdog_rearm_us > PTPD_RX_STARVATION_US) {
        (void)ptp_port_reopen_l2tap(state, 0);
        /* Rearm on our own clock regardless of outcome so a dead link
         * doesn't spin the reopen path faster than once per window. */
        watchdog_rearm_us = now_us;
      }
    }

    if (ptp_ftm_daemon_tick) ptp_ftm_daemon_tick();
    bool source_was_valid = state->selected_source_valid;
    state->selected_source_valid = is_selected_source_valid(state);
    if (source_was_valid && !state->selected_source_valid) {
      if (ptp_is_gptp(state) && state->selected_source.btc_priority1 < 255 &&
          !ptp_sync_receipt_current(&state->port[0].sync_receipt, esp_timer_get_time()))
        ptpinfo("SYNCTIMEOUT,%" PRId64 ",%" PRId64 ",%" PRId64,
            esp_timer_get_time(), state->port[0].sync_receipt.received_us,
            state->port[0].sync_receipt.interval_us);
      ptp_invalidate_timing();
    }
    ptp_publish_local_source(state);
    ptp_refresh_link_speed(state);
    ptp_apply_rx_timestamp_mode(state);
    ptp_periodic_send(state);
    ptp_check_profile_fallback(state);
    ptp_process_statusreq(state);
  } // while (!state->stop)
  portENTER_CRITICAL(&s_injected_lock);
  s_injected_enabled = false;
  s_injected_count = 0;
  for (unsigned port = 0; port < CONFIG_ESP_PTP_NUM_PORTS; ++port)
    ++s_injected_generation[port];
  portEXIT_CRITICAL(&s_injected_lock);
  ptp_destroy_state(state);
  free(state);

err:
  s_state = NULL;
  vTaskDelete(NULL);
}

/****************************************************************************
 * Public Functions
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

int ptpd_start_port(int port_index, FAR const char *interface,
                    ptp_port_medium_e medium) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    ESP_LOGE(TAG, "ptpd_start_port: port_index %d out of range [0,%d)",
             port_index, CONFIG_ESP_PTP_NUM_PORTS);
    return -EINVAL;
  }
  if (medium != ptp_port_medium_eth_hwts &&
      medium != ptp_port_medium_wifi_ftm) {
    ESP_LOGE(TAG, "ptpd_start_port: medium %d not supported", (int)medium);
    return -ENOSYS;
  }

  /* The branch is decided by whether the daemon is already running,
   * not by port_index. Either branch handles either medium.
   *
   * Bootstrap branch (s_state == NULL): spawn the daemon task with a
   * heap-allocated ptp_bootstrap_args_s. ptp_initialize_state on the
   * task side dispatches to the per-medium init helper for the
   * addressed port.
   *
   * Attach branch (s_state != NULL): call the per-medium init helper
   * directly on the running daemon. For eth_hwts this opens an
   * additional L2TAP socket; for wifi_ftm it reads the radio MAC and
   * leaves the Sync transport for the egress callback to drive. */
  if (s_state == NULL) {
    struct ptp_bootstrap_args_s *args = calloc(1, sizeof(*args));
    if (args == NULL) {
      ESP_LOGE(TAG, "ptpd_start_port: out of memory");
      return -ENOMEM;
    }
    args->port_index = port_index;
    args->medium = medium;
    if (interface != NULL) {
      strncpy(args->interface, interface, sizeof(args->interface) - 1);
    }
    /* On dual-core targets (e.g. ESP32-P4), pin PTPD to core 1 at
     * priority 22 (same band as esp_timer/emac_rx, below sdio/rpc/wifi
     * at 23). Core 1 is otherwise near-idle on the bridge build, so
     * PTPD never competes with the SoftAP/SDIO storm on core 0 —
     * keeps Pdelay_Resp turnaround sub-millisecond. On single-core
     * targets (ESP32-C6), use tskNO_AFFINITY since core 1 doesn't
     * exist and xTaskCreatePinnedToCore would assert. */
#if portNUM_PROCESSORS > 1
    const BaseType_t ptpd_core = 1;
#else
    const BaseType_t ptpd_core = tskNO_AFFINITY;
#endif
    if (xTaskCreatePinnedToCore(ptp_daemon, "PTPD",
                                CONFIG_ESP_PTP_STACKSIZE, args, 22, NULL,
                                ptpd_core) != pdPASS) {
      free(args);
      ESP_LOGE(TAG, "ptpd_start_port: xTaskCreate failed");
      return -1;
    }
    return 1;
  }

  /* Attach to running daemon. */
  struct ptp_port_s *p = &s_state->port[port_index];
  p->enabled = true;
  p->link_up = true; /* updated by event handlers */
  p->medium = medium;
  if (interface != NULL) {
    strncpy(p->interface_name, interface, sizeof(p->interface_name) - 1);
    p->interface_name[sizeof(p->interface_name) - 1] = '\0';
  } else {
    p->interface_name[0] = '\0';
  }

  int rc;
  switch (medium) {
  case ptp_port_medium_eth_hwts:
    rc = ptp_port_init_eth_hwts(s_state, port_index, p->interface_name);
    break;
  case ptp_port_medium_wifi_ftm:
    rc = ptp_port_init_wifi_ftm(s_state, port_index, p->interface_name);
    break;
  default:
    rc = -ENOSYS;
    break;
  }
  if (rc != OK) {
    p->enabled = false;
    return rc;
  }

  ESP_LOGI(TAG,
           "ptpd_start_port: port=%d iface=\"%s\" medium=%s attached "
           "(wifi_mode=%s)",
           port_index, p->interface_name,
           medium == ptp_port_medium_eth_hwts ? "eth_hwts" : "wifi_ftm",
           p->wifi_mode == ptp_port_wifi_mode_ap    ? "ap"
           : p->wifi_mode == ptp_port_wifi_mode_sta ? "sta"
                                                    : "none");
  return port_index;
}

int ptpd_inject_peer_delay(int port_index, int64_t peer_delay_ns) {
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  /* Discard out-of-band samples (multipath / handoff). */
  if (peer_delay_ns < 0 ||
      peer_delay_ns >= CONFIG_ESP_PTP_MAX_PEER_DELAY_NS) {
    return -ERANGE;
  }
  struct ptp_port_s *p = &s_state->port[port_index];

  /* Running average over DELAYREQ_AVGCOUNT samples; smoothed value
   * read via port->peer_delay_ns. */
  if (p->peer_delay_avgcount < CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT) {
    p->peer_delay_avgcount++;
  }
  p->peer_delay_ns +=
      (long)(peer_delay_ns - p->peer_delay_ns) / p->peer_delay_avgcount;
  return OK;
}

/* Queue a Wi-Fi PTP frame for daemon-owned selection and servo processing.
 * Preserve the callback-time software timestamp, not the dequeue time.
 * Timing handlers still use logical port zero; Signaling retains ingress. */
int ptp_inject_received_frame_from(int port_index, const uint8_t *frame,
                                  uint16_t len, const uint8_t source_mac[6]) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS ||
      !frame || len < sizeof(struct ptp_header_s) ||
      len > sizeof(((ptp_msgbuf *)0)->raw)) return -EINVAL;
  portENTER_CRITICAL(&s_injected_lock);
  uint32_t generation = s_injected_generation[port_index];
  portEXIT_CRITICAL(&s_injected_lock);
  struct timespec received;
  if (ptp_gettime(NULL, &received) != 0) return -EIO;
  portENTER_CRITICAL(&s_injected_lock);
  int result = OK;
  if (!s_injected_enabled) {
    result = -ESRCH;
  } else if (generation != s_injected_generation[port_index]) {
    result = -ESTALE;
  } else if (s_injected_count == PTP_INJECT_DEPTH) {
    result = -ENOBUFS;
  } else {
    unsigned tail = (s_injected_head + s_injected_count) % PTP_INJECT_DEPTH;
    s_injected[tail].length = len;
    s_injected[tail].port_index = (uint8_t)port_index;
    s_injected[tail].received = received;
    s_injected[tail].link_generation = generation;
    s_injected[tail].source_mac_valid = source_mac != NULL;
    memset(s_injected[tail].source_mac, 0, 6);
    if (source_mac) memcpy(s_injected[tail].source_mac, source_mac, 6);
    memset(&s_injected[tail].message, 0, sizeof(ptp_msgbuf));
    memcpy(s_injected[tail].message.raw, frame, len);
    ++s_injected_count;
  }
  portEXIT_CRITICAL(&s_injected_lock);
  return result;
}

int ptp_inject_received_frame(int port_index, const uint8_t *frame, uint16_t len)
{
  return ptp_inject_received_frame_from(port_index, frame, len, NULL);
}

int ptpd_inject_sync(int port_index, FAR const uint8_t *follow_up_info,
                     size_t len) {
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  if (follow_up_info == NULL || len < sizeof(struct ptp_follow_up_s)) {
    return -EINVAL;
  }

  /* The §12.7 IE payload IS an entire 802.1AS Follow_Up message.
   * RX instant is "now" (t2); preciseOriginTimestamp is t1. */
  const struct ptp_follow_up_s *fu =
      (const struct ptp_follow_up_s *)follow_up_info;

  /* Validate messagetype (low nibble = Follow_Up = 0x08; high nibble
   * carries gPTP majorSdoId on gPTP messages). */
  if ((fu->header.messagetype & PTP_MSGTYPE_MASK) != PTP_MSGTYPE_FOLLOW_UP) {
    return -EINVAL;
  }

  struct ptp_port_s *p = &s_state->port[port_index];

  /* Synthesise selected_source on first injection from the Follow_Up
   * alone — the real BTC priority / clockQuality arrives separately
   * via §12.2 unicast Announce frames (handled by the normal
   * ptp_process_announce path through ptp_inject_received_frame). */
  if (!s_state->selected_source_valid ||
      memcmp(s_state->selected_source.header.sourceidentity,
             fu->header.sourceidentity, 8) != 0) {
    memset(&s_state->selected_source, 0, sizeof(s_state->selected_source));
    s_state->selected_source.header = fu->header;
    memcpy(s_state->selected_source.btc_identity, fu->header.sourceidentity,
           sizeof(s_state->selected_source.btc_identity));
    s_state->selected_source_valid = true;
    ptpinfo("inject_sync port=%d: locked onto GM "
            "%02x%02x%02x%02x%02x%02x%02x%02x\n",
            port_index, fu->header.sourceidentity[0],
            fu->header.sourceidentity[1], fu->header.sourceidentity[2],
            fu->header.sourceidentity[3], fu->header.sourceidentity[4],
            fu->header.sourceidentity[5], fu->header.sourceidentity[6],
            fu->header.sourceidentity[7]);
  }

  /* Decode preciseOriginTimestamp (10 B, 6 sec + 4 nsec). */
  struct timespec remote_time;
  ptp_format_to_timespec(fu->origintimestamp, &remote_time);

  /* Local RX instant — best we can do without HW timestamping the
   * beacon (a bounded penalty, see §12 carrier-deviation note). */
  struct timespec local_rxtime;
  ptp_gettime(s_state, &local_rxtime);

  clock_gettime(CLOCK_MONOTONIC, &p->last_received_sync);

  return ptp_update_local_clock(s_state, &remote_time, &local_rxtime);
}

int ptpd_inject_sync_pair(int port_index, int64_t remote_ns, int64_t local_ns) {
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  struct ptp_port_s *p = &s_state->port[port_index];

  /* Synthesise a selected_source if none yet — without it the
   * timereceiver-role tests reject the injection. Identity was
   * validated upstream at the §12.7 IE parse layer. */
  if (!s_state->selected_source_valid) {
    return -ENOENT;
  }

  struct timespec remote_ts = {.tv_sec = (time_t)(remote_ns / NSEC_PER_SEC),
                               .tv_nsec = (long)(remote_ns % NSEC_PER_SEC)};
  struct timespec local_ts = {.tv_sec = (time_t)(local_ns / NSEC_PER_SEC),
                              .tv_nsec = (long)(local_ns % NSEC_PER_SEC)};

  clock_gettime(CLOCK_MONOTONIC, &p->last_received_sync);

  return ptp_update_local_clock(s_state, &remote_ts, &local_ts);
}

int ptpd_register_sync_egress_cb(int port_index, ptpd_sync_egress_cb_t cb,
                                 FAR void *ctx) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  if (s_state == NULL) {
    return -ESRCH;
  }
  s_state->port[port_index].sync_egress_cb = cb;
  s_state->port[port_index].sync_egress_ctx = ctx;
  return OK;
}

/* When ptp_clock_sw_init() registers a backend, ptpd_now() routes
 * through it; otherwise falls back to clock_gettime(). */
typedef int (*ptpd_sw_clock_now_fn)(struct timespec *ts);
static ptpd_sw_clock_now_fn s_sw_clock_now = NULL;

void ptpd_set_sw_clock_now(ptpd_sw_clock_now_fn fn) { s_sw_clock_now = fn; }

int ptpd_now(FAR struct timespec *ts) {
  if (s_sw_clock_now != NULL) {
    return s_sw_clock_now(ts);
  }
  return clock_gettime(PTPD_CLOCK_ID, ts);
}

int ptpd_start(FAR const char *interface) {
  if (s_state != NULL) {
    ESP_LOGE(TAG, "Other instance of PTP is already running");
    return -1;
  }
  struct ptp_bootstrap_args_s *args = calloc(1, sizeof(*args));
  if (args == NULL) {
    ESP_LOGE(TAG, "ptpd_start: out of memory");
    return -ENOMEM;
  }
  args->port_index = 0;
  args->medium = ptp_port_medium_eth_hwts;
  if (interface != NULL) {
    strncpy(args->interface, interface, sizeof(args->interface) - 1);
  }
  if (xTaskCreate(ptp_daemon, "PTPD", CONFIG_ESP_PTP_STACKSIZE, args, 6,
                  NULL) != pdPASS) {
    free(args);
    ESP_LOGE(TAG, "ptpd_start: xTaskCreate failed");
    return -1;
  }
  return 1;
}

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
 *   Multiple threads with priority less than CONFIG_ESP_PTP_SERVERPRIO
 *   can request status simultaneously. If higher priority threads request
 *   status simultaneously, some of the requests may timeout.
 *
 ****************************************************************************/

int ptpd_set_profile(int pid, ptp_profile_e profile) {
  UNUSED(pid);

  if (profile != ptp_profile_standard && profile != ptp_profile_gptp) {
    return -EINVAL;
  }

  if (s_state == NULL) {
    return -ESRCH;
  }

  if (s_state->active_ptp_profile != profile) {
    s_state->active_ptp_profile = profile;
    s_state->preferred_ptp_profile = profile; /* an explicit set is the new intent */
    ptp_reset_for_profile(s_state);
    if (profile == ptp_profile_gptp) {
      ptp_arm_profile_fallback(s_state);
    }
    ptpinfo("PTP profile changed to %s mode.\n",
            profile == ptp_profile_gptp ? "gPTP" : "standard");
  }

  return OK;
}

int ptpd_status(int pid, FAR struct ptpd_status_s *status) {
  int ret = 0;
  sem_t donesem;
  struct ptpd_statusreq_s req;
  struct timespec timeout;

  /* Defend against callers that ask for status before the daemon
   * has been started. ptpd_start() sets s_state; builds that don't
   * call it (e.g. a c6 wireless endpoint using the software-clock
   * fallback without the protocol loop) would otherwise dereference
   * NULL. Callers already skip the status field on nonzero return. */
  if (s_state == NULL) {
    return -ENODEV;
  }

  /* Fill in the status request */

  memset(status, 0, sizeof(struct ptpd_status_s));
  sem_init(&donesem, 0, 0);
  req.done = &donesem;
  req.dest = status;

  s_state->status_req = req;

  /* Wait for status request to be handled */
  clock_gettime(CLOCK_REALTIME, &timeout); // sem_timedwait uses CLOCK_REALTIME
  timeout.tv_sec += 1;

  if (sem_timedwait(&donesem, &timeout) != 0) {
    req.done = NULL;
    req.dest = NULL;
    s_state->status_req = req;
    ret = -errno;
  }
  sem_destroy(&donesem);

  return ret;
#ifndef CONFIG_BUILD_FLAT

  return -ENOTSUP;

#else

  int ret = OK;
  sem_t donesem;
  struct ptpd_statusreq_s req;
  union sigval val;
  struct timespec timeout;

  /* Fill in the status request */

  memset(status, 0, sizeof(struct ptpd_status_s));
  sem_init(&donesem, 0, 0);
  req.done = &donesem;
  req.dest = status;
  val.sival_ptr = &req;

  if (sigqueue(pid, SIGUSR1, val) != OK) {
    return -errno;
  }

  /* Wait for status request to be handled */

  clock_gettime(CLOCK_MONOTONIC, &timeout);
  timeout.tv_sec += 1;
  if (sem_clockwait(&donesem, CLOCK_MONOTONIC, &timeout) != 0) {
    ret = -errno;
  }

  return ret;

#endif /* CONFIG_BUILD_FLAT */
}

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

int ptpd_stop(int pid) {
  s_state->stop = true;
  return OK;
}

bool ptpd_port_link_up(int port_index) {
  if (!s_state) {
    /* Daemon not started yet — treat as up so callers do not block TX
     * during their own bring-up window. The first link event after
     * ptpd_start will correct any divergence. */
    return true;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return false;
  }
  const struct ptp_port_s *p = &s_state->port[port_index];
  return p->enabled && p->link_up;
}
