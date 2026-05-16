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

#define ESP_PTP 1

/****************************************************************************
 * Included Files
 ****************************************************************************/
#ifndef ESP_PTP
#include <nuttx/config.h>
#endif

#include <stdbool.h>
#include <stdint.h>

#include <sys/socket.h>
#include <sys/time.h>

#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifndef ESP_PTP
#include <debug.h>
#endif
#include <fcntl.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#ifndef ESP_PTP
#include <netutils/ipmsfilter.h>
#endif

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#ifndef ESP_PTP
#include <netutils/ptpd.h>
#include <nuttx/net/netconfig.h>
#endif

#include "ptp.h"
#include "ptp_clock_sw.h"

/* Set true after ptp_clock_sw_init() succeeds — routes ptp_gettime /   */
/* ptp_settime / ptp_adjtime and the freq-adjust path through the SW   */
/* clock backend instead of POSIX. Chips without EMAC IEEE 1588        */
/* (e.g. ESP32-C6) have no working clock_adjtime(CLOCK_REALTIME,       */
/* ADJ_FREQUENCY) in IDF; the SW backend re-anchors against            */
/* esp_timer_get_time() with a rate offset and is the only way the     */
/* servo can actually move the local clock rate on those chips.        */
static bool s_use_sw_clock = false;

#ifdef ESP_PTP
#include "esp_err.h"
#include "esp_eth_driver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ptp.h"
#include "esp_vfs_l2tap.h"
#include "esp_wifi.h"
#include "lwip/prot/ethernet.h" // Ethernet headers
#include "semaphore.h"
#include <math.h>

#include "esp_timer.h"
#include "soc/soc_caps.h"
#include <sys/timex.h>

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
#define PTPD_HAVE_ESP_ETH_CLOCK 1
#define PTPD_CLOCK_ID CLOCK_PTP_SYSTEM
#else
#define PTPD_HAVE_ESP_ETH_CLOCK 0
#define PTPD_CLOCK_ID CLOCK_REALTIME
#endif

#define ETH_TYPE_PTP 0x88F7

/* Lateness diagnostic — detects when ptpd is starved by higher-priority
 * tasks. If pdelay exchange misses exceed 3 in a row the upstream bridge
 * declares the link asCapable=false and stops forwarding sync. */
static int64_t s_ptpd_last_loop_us = 0;
static int64_t s_ptpd_loop_max_gap_us = 0;
static int64_t s_ptpd_last_report_us = 0;
static uint32_t s_ptpd_loop_iters = 0;
static uint32_t s_ptpd_tx_req_total = 0;
static uint32_t s_ptpd_tx_req_late = 0;
static int64_t s_ptpd_tx_req_late_max_us = 0;

/* Per-message-type RX counters. Incremented in ptp_process_rx_packet for
 * each successfully-dispatched packet. Reset per report window so we can
 * compare against wire captures: expected ~80 syncs + ~80 follow-ups +
 * ~10 announces + ~1 pdelay_req/resp/fup each over a 10 s window. */
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
  ESP_LOGW(
      "ptpd-late",
      "iters=%u loop_gap_max=%lldms tx=%u/%u late(max=%lldms) "
      "rx_sync=%u max_gap=%lldms fup=%u ann=%u pd_req=%u pd_resp=%u pd_fup=%u",
      (unsigned)s_ptpd_loop_iters, s_ptpd_loop_max_gap_us / 1000,
      (unsigned)s_ptpd_tx_req_late, (unsigned)s_ptpd_tx_req_total,
      s_ptpd_tx_req_late_max_us / 1000, (unsigned)s_ptpd_rx_sync,
      s_ptpd_rx_sync_max_gap_us / 1000, (unsigned)s_ptpd_rx_followup,
      (unsigned)s_ptpd_rx_announce, (unsigned)s_ptpd_rx_pdelay_req,
      (unsigned)s_ptpd_rx_pdelay_resp, (unsigned)s_ptpd_rx_pdelay_fup);
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

// gPTP required values
#define GPTP_DELAYREQ_INTERVAL_MS 1000
#define CONFIG_CLOCK_ADJTIME_PERIOD_MS (CONFIG_ETH_CLOCK_ADJTIME_PERIOD_MS)
#define CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM                                     \
  (CONFIG_ETH_CLOCK_ADJTIME_SLEWLIMIT_PPB / 1000)

// To able to set either only server or only client
#ifndef CONFIG_NETUTILS_PTPD_TIMEOUT_MS
#define CONFIG_NETUTILS_PTPD_TIMEOUT_MS 0
#endif
#ifndef CONFIG_NETUTILS_PTPD_SETTIME_THRESHOLD_MS
#define CONFIG_NETUTILS_PTPD_SETTIME_THRESHOLD_MS 0
#endif
#ifndef CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS
#define CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS 0
#endif
#ifndef CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT
#define CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT 0
#endif
#ifndef CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS
#define CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS 16000
#endif
#ifndef CONFIG_NETUTILS_PTPD_PATH_DELAY_STABILITY_NS
#define CONFIG_NETUTILS_PTPD_PATH_DELAY_STABILITY_NS 250
#endif
#ifndef CONFIG_NETUTILS_PTPD_PEER_DELAY_STABILITY_NS
#define CONFIG_NETUTILS_PTPD_PEER_DELAY_STABILITY_NS 100
#endif
#ifndef CONFIG_NETUTILS_PTPD_MAX_PEER_DELAY_NS
#define CONFIG_NETUTILS_PTPD_MAX_PEER_DELAY_NS 100000LL // 100us
#endif

#define clock_timespec_subtract(ts1, ts2, ts3) timespecsub(ts1, ts2, ts3)
#define clock_timespec_add(ts1, ts2, ts3) timespecadd(ts1, ts2, ts3)

#endif // ESP_PTP

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef ESP_PTP
#define ADJ_FREQ_MAX 512000 // TODO tuneup
typedef struct {
  int32_t kp;
  int32_t ki;
  int32_t drift_acc;
} pi_cntrl_t;
#endif // ESP_PTP

typedef union {
  struct ptp_header_s header;
  struct ptp_announce_s announce;
  struct ptp_sync_s sync;
  struct ptp_follow_up_s follow_up;
  struct ptp_delay_req_s delay_req;
  struct ptp_delay_resp_s delay_resp;
  struct ptp_delay_resp_follow_up_s delay_resp_follow_up;
  uint8_t raw[128];
} ptp_msgbuf;

/* Carrier structure for querying PTPD status */

struct ptpd_statusreq_s {
  FAR sem_t *done;
  FAR struct ptpd_status_s *dest;
};

/* Main PTPD state storage */

/* Multi-port scaffolding. Per-port state lives in ptp_port_s; per-port
 * accesses go through state->port[i]. With CONFIG_ESP_PTP_NUM_PORTS=1
 * (the default) the only port is port[0] and its config is fixed to
 * { ethernet, gptp_wire } so historical single-port behavior is
 * preserved.
 *
 * esp_ptp must stand alone — no identifier in this struct refers to any
 * higher-level protocol by name. (The intended consumer is an AVB stack,
 * but that context is captured in comments only.) */

#define PTP_PDELAY_RESP_MAX_TRACKED 4

/* Bootstrap arguments passed from ptpd_start / ptpd_start_port to the
 * daemon task on creation. Heap-allocated by the start function,
 * consumed and freed by ptp_daemon after ptp_initialize_state copies
 * what it needs. Encodes which port the daemon is bootstrapping on,
 * its medium, and the interface label so ptp_initialize_state can
 * dispatch to the correct per-medium init helper without baking in
 * any port-index assumption. */
struct ptp_bootstrap_args_s {
  int port_index;
  ptp_port_medium_e medium;
  char interface[16];
};

struct ptp_port_s {
  /* Configuration. */
  bool enabled;
  ptp_port_medium_e medium;
  ptp_port_host_if_e host_if;       /* how this port attaches to the SoC */
  ptp_port_type_e type;             /* primary / failover / bridged */
  ptp_port_wifi_mode_e wifi_mode;   /* ap/sta for medium=wifi_ftm, none otherwise */
  uint32_t link_speed_mbps;         /* nominal PHY-rate cap */
  char interface_name[16];

  /* Egress callback for ports whose Sync transport is out-of-band
   * (e.g. an AP that publishes FollowUpInformation in a beacon Vendor
   * IE). NULL on ports that use on-wire IEEE 1588 / 802.1AS frames. */
  ptpd_sync_egress_cb_t sync_egress_cb;
  FAR void *sync_egress_ctx;

  /* Per-port runtime state. */
#ifdef ESP_PTP
  uint8_t intf_hw_addr[ETH_ADDR_LEN];
  int ptp_socket;
#endif

  /* Last received packet timestamps (CLOCK_MONOTONIC). */
  struct timespec last_received_multicast;
  struct timespec last_received_announce;
  struct timespec last_received_sync;

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
  unsigned int pdelay_req_attempts_unanswered;

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
  ptp_msgbuf rxbuf;

  /* Buffered sync packet for two-step clock setting (server sends the
   * accurate timestamp in a separate follow-up message). */
  struct ptp_sync_s twostep_packet;
  struct timespec twostep_rxtime;

  /* Buffered delay_resp packet for two-step peer delay measurement. */
  struct ptp_delay_resp_s twostep_delay_resp_packet;
  struct timespec twostep_delay_resp_rxtime;
};

struct ptp_state_s {
  /* Request for PTPD task to stop or report status */

  bool stop;
  struct ptpd_statusreq_s status_req;

  /* Per-port array. */

  struct ptp_port_s port[CONFIG_ESP_PTP_NUM_PORTS];

#ifdef ESP_PTP
  ptp_profile_e ptp_profile;

  int64_t remote_time_ns_prev;
  int64_t local_time_ns_prev;

  int64_t last_offset_ns;
  double correction_ns;

  pi_cntrl_t offset_pi;
#else
  /* Address of network interface we are operating on */

  struct sockaddr_in interface_addr;

  /* Socket bound to interface for transmission */

  int tx_socket;

  /* Sockets for PTP event and information ports */

  int event_socket;

  int info_socket;
#endif // ESP_PTP

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

  /* gPTP → standard PTP fallback gate. One-shot per session: set true after
   * either AVB Lite §2.2 condition fires; cleared on Ethernet link-up so the
   * check re-arms for the new link. */

  bool gptp_fallback_done;
  bool eth_event_handler_registered;

#ifndef ESP_PTP
  uint8_t rxcmsg[CMSG_LEN(sizeof(struct timeval))];
#endif // ESP_PTP
};

#ifdef CONFIG_NETUTILS_PTPD_SERVER
#define PTPD_POLL_INTERVAL CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MS
#else
#define PTPD_POLL_INTERVAL CONFIG_NETUTILS_PTPD_TIMEOUT_MS
#endif

/* PTP debug messages are enabled by either CONFIG_DEBUG_NET_INFO
 * or separately by CONFIG_NETUTILS_PTPD_DEBUG. This simplifies
 * debugging without having excessive amount of logging from net.
 */

#ifdef ESP_PTP
static const char *TAG = "ptpd";
#define ptpinfo(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define ptpwarn(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define ptperr(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#else
#ifdef CONFIG_NETUTILS_PTPD_DEBUG
#define ptpinfo _info
#define ptpwarn _warn
#define ptperr _err
#else
#define ptpinfo ninfo
#define ptpwarn nwarn
#define ptperr nerr
#endif
#endif // ESP_PTP

#ifdef ESP_PTP
static struct ptp_state_s *s_state;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef ESP_PTP
static void ptp_clean_after_step(FAR struct ptp_state_s *state);

static inline bool ptp_is_gptp(FAR const struct ptp_state_s *state) {
  return state->ptp_profile == ptp_profile_gptp;
}

static void ptp_arm_profile_fallback(FAR struct ptp_state_s *state) {
  if (!ptp_is_gptp(state)) {
    return;
  }

  state->gptp_fallback_done = false;
  state->port[0].last_transmitted_delayreq.tv_sec = 0;
  state->port[0].last_transmitted_delayreq.tv_nsec = 0;

  /* AVB Lite fallback re-evaluation on link-up (profiles/avb_lite.md §2.2). */

  state->port[0].peer_is_endpoint = false;
  state->port[0].pdelay_req_attempts_unanswered = 0;
  state->port[0].pdelay_resp_responder_count = 0;
  state->port[0].pdelay_multi_responder = false;
  memset(state->port[0].pdelay_resp_responders, 0,
         sizeof(state->port[0].pdelay_resp_responders));
  state->port[0].last_endpoint_beacon.tv_sec = 0;
  state->port[0].last_endpoint_beacon.tv_nsec = 0;

  ptpinfo("Armed profile fallback evaluation\n");
}

static void ptp_eth_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
  UNUSED(event_base);
  UNUSED(event_data);
  FAR struct ptp_state_s *state = (FAR struct ptp_state_s *)arg;

  if (!state) {
    return;
  }

  if (event_id == ETHERNET_EVENT_CONNECTED) {
    /* A link-up event starts a new "startup" window. Resume in the current
     * profile; if that profile is gPTP, require one PDelay_Resp again. */
    ptp_arm_profile_fallback(state);
  } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
  }
}

static void ptp_reset_for_profile(FAR struct ptp_state_s *state) {
  state->selected_source_valid = false;
  memset(&state->selected_source, 0, sizeof(state->selected_source));
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
    state->port[0].delayreq_interval_ms = GPTP_DELAYREQ_INTERVAL_MS;
    state->port[0].next_delayreq_interval_ms = GPTP_DELAYREQ_INTERVAL_MS;
  } else {
    state->port[0].delayreq_interval_ms =
        CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS;
    state->port[0].next_delayreq_interval_ms =
        CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS;
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
}

static void ptp_create_eth_frame(struct ptp_state_s *state, uint8_t *eth_frame,
                                 void *ptp_msg, uint16_t ptp_msg_len) {
  ptp_create_eth_frame_to(state, eth_frame, ptp_msg, ptp_msg_len,
                          ptp_is_gptp(state) ? LLDP_MULTICAST_ADDR
                                             : PTP4L_MULTICAST_ADDR);
}

static int ptp_net_send_to(FAR struct ptp_state_s *state, void *ptp_msg,
                           uint16_t ptp_msg_len, struct timespec *ts,
                           const uint8_t *dest_mac) {
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
  uint8_t eth_frame[ptp_msg_len + ETH_HEADER_LEN];

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

  // check if read was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;
  }

  memcpy(ptp_msg, &eth_frame[ETH_HEADER_LEN], ret);

  return ret;
}

static int64_t timespec_to_ns(FAR const struct timespec *ts) {
  return ts->tv_sec * NSEC_PER_SEC + (ts->tv_nsec);
}
#endif // ESP_PTP

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
  int system_identity_check = 1;

  if (a->btc_priority1 < b->btc_priority1      /* Main priority field */
      || a->btc_quality[0] < b->btc_quality[0] /* Clock class */
      || a->btc_quality[1] < b->btc_quality[1] /* Clock accuracy */
      || a->btc_quality[2] < b->btc_quality[2] /* Clock variance high byte */
      || a->btc_quality[3] < b->btc_quality[3] /* Clock variance low byte */
      || a->btc_priority2 < b->btc_priority2   /* Sub priority field */
      || memcmp(a->btc_identity, b->btc_identity, sizeof(a->btc_identity)) < 0) {
    system_identity_check = -1;
  } else if (a->btc_priority1 == b->btc_priority1      /* Main priority field */
             && a->btc_quality[0] == b->btc_quality[0] /* Clock class */
             && a->btc_quality[1] == b->btc_quality[1] /* Clock accuracy */
             &&
             a->btc_quality[2] == b->btc_quality[2] /* Clock variance high byte */
             &&
             a->btc_quality[3] == b->btc_quality[3]  /* Clock variance low byte */
             && a->btc_priority2 == b->btc_priority2 /* Sub priority field */
             && memcmp(a->btc_identity, b->btc_identity,
                       sizeof(a->btc_identity)) == 0) {
    system_identity_check = 0;
  }

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

/* Check if the currently selected source is still valid */

static bool is_selected_source_valid(FAR struct ptp_state_s *state) {
  struct timespec time_now;
  struct timespec delta;

  if ((state->selected_source.header.messagetype & PTP_MSGTYPE_MASK) !=
      PTP_MSGTYPE_ANNOUNCE) {
    return false; /* Uninitialized value */
  }

  /* Note: this uses monotonic clock to track the timeout even when
   *       system clock is adjusted.
   */

  clock_gettime(CLOCK_MONOTONIC, &time_now);
  clock_timespec_subtract(&time_now, &state->port[0].last_received_sync,
                          &delta);

  if (timespec_to_ms(&delta) > CONFIG_NETUTILS_PTPD_TIMEOUT_MS) {
#ifdef ESP_PTP
    ESP_LOGD(TAG, "Too long time since received packet\n");
#endif            // ESP_PTP
    return false; /* Too long time since received packet */
  }

  return true;
}

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
#ifdef ESP_PTP
  if (s_use_sw_clock) {
    return ptp_clock_sw_adjtime_offset(delta_ns);
  }
  struct timex tx = {
      .modes = ADJ_OFFSET | ADJ_NANO,
      .offset = (long)delta_ns,
  };
  return clock_adjtime(PTPD_CLOCK_ID, &tx);
#else
  struct timeval delta;
  delta.tv_sec = delta_ns / NSEC_PER_SEC;
  delta_ns -= (int64_t)delta.tv_sec * NSEC_PER_SEC;
  delta.tv_usec = delta_ns / NSEC_PER_USEC;
  return adjtime(&delta, NULL);
#endif
}

#ifndef ESP_PTP
/* Get timestamp of latest received packet */

static int ptp_getrxtime(FAR struct ptp_state_s *state,
                         FAR struct msghdr *rxhdr, FAR struct timespec *ts) {
  /* Get hardware or kernel timestamp if available */

#ifdef CONFIG_NET_TIMESTAMP
  struct cmsghdr *cmsg;

  for_each_cmsghdr(cmsg, rxhdr) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP &&
        cmsg->cmsg_len == CMSG_LEN(sizeof(struct timeval))) {
      TIMEVAL_TO_TIMESPEC((FAR struct timeval *)CMSG_DATA(cmsg), ts);

      /* Sanity-check the value */

      if (ts->tv_sec > 0 || ts->tv_nsec > 0) {
        return OK;
      }
    }
  }

  ptpwarn("CONFIG_NET_TIMESTAMP enabled but did not get packet timestamp\n");
#endif

  /* Fall back to current timestamp */

  return ptp_gettime(state, ts);
}
#endif // !ESP_PTP

/* Translate the per-port Kconfig topology knobs (MEDIUM / HOST_IF /
 * TYPE / WIFI_MODE / LINK_SPEED_MBPS) into the runtime ptp_port_s.
 * Called from ptp_initialize_state after port[0] has been seeded with
 * interface/socket. esp_avb reads these same Kconfig knobs via its
 * own mirror — esp_ptp keeps its copy so internal decisions (timestamp
 * source, sync-interval scaling for non-EMAC ports) can use them
 * without depending on esp_avb. */
#ifdef ESP_PTP
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
#endif /* ESP_PTP */

#ifdef ESP_PTP

/* Per-medium port initialisation. Each helper assumes the caller has
 * already filled the port's enabled/medium/interface_name/wifi_mode
 * fields (the wifi_mode comes from ptp_apply_port_topology_kconfig).
 * The helper performs medium-specific work — opening sockets and
 * registering driver hooks for eth_hwts, fetching the radio MAC for
 * wifi_ftm — and writes intf_hw_addr. Used by ptp_initialize_state on
 * the bootstrap port AND by ptpd_start_port's attach branch for
 * additional ports added to a running daemon. */

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
    ptperr("port %d: failed to set L2TAP ethertype filter: %d\n",
           port_index, errno);
    return ERROR;
  }
  esp_eth_handle_t eth_handle;
  if (ioctl(p->ptp_socket, L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
    ptperr("port %d: failed to fetch eth_handle from L2TAP: %d\n",
           port_index, errno);
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
    ptperr("port %d: failed to enable L2TAP timestamping: %d\n",
           port_index, errno);
    return ERROR;
  }

  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &p->intf_hw_addr);

  uint8_t dest_addr[ETH_ADDR_LEN];
  SET_MAC_ADDR(dest_addr, 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00);
  esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);
  SET_MAC_ADDR(dest_addr, 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E);
  esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, dest_addr);

  /* Eth-link events drive gPTP link-up fallback detection. The
   * handler is registered once per daemon (re-registration on a
   * second eth_hwts port would be a no-op or error from the event
   * loop); the eth_event_handler_registered flag prevents double-
   * registration when more than one eth_hwts port comes up. */
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

  return OK;
}

/* Initialise daemon-wide state and bootstrap the addressed port. The
 * port_index, medium and interface come from the heap-allocated
 * ptp_bootstrap_args_s the caller (ptpd_start / ptpd_start_port)
 * handed to the daemon task. */
static int ptp_initialize_state(FAR struct ptp_state_s *state,
                                FAR const struct ptp_bootstrap_args_s *args) {
  if (args->port_index < 0 ||
      args->port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    ptperr("bootstrap port_index %d out of range [0,%d)\n",
           args->port_index, CONFIG_ESP_PTP_NUM_PORTS);
    return ERROR;
  }

  /* Daemon-wide state: servo, profile, time baselines. */
  state->remote_time_ns_prev = 0;
  state->local_time_ns_prev = 0;
  state->offset_pi.kp = 1;
  state->offset_pi.ki = 3; /* matches ptp4l default gain ~0.3 */
  state->offset_pi.drift_acc = 0;

#ifdef CONFIG_NETUTILS_PTPD_GPTP_PROFILE
  state->ptp_profile = ptp_profile_gptp;
#else
  state->ptp_profile = ptp_profile_standard;
#endif
  ptp_reset_for_profile(state);

  /* Per-port topology (host_if, type, wifi_mode, link_speed_mbps) for
   * ALL ports from Kconfig. esp_avb mirrors these knobs; esp_ptp's
   * copy informs ptpd's own decisions (timestamp accuracy, sync
   * cadence). Runs before per-medium init so the helpers can read
   * wifi_mode etc. on the bootstrap port. */
  ptp_apply_port_topology_kconfig(state);

  /* Seed the bootstrap port's API-level config — the runtime fields
   * (intf_hw_addr, ptp_socket) are filled by the medium helper. */
  struct ptp_port_s *p = &state->port[args->port_index];
  p->enabled = true;
  p->medium = args->medium;
  strncpy(p->interface_name, args->interface,
          sizeof(p->interface_name) - 1);
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
  state->own_identity.header.domain = CONFIG_NETUTILS_PTPD_DOMAIN;
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
#if defined(CONFIG_NETUTILS_PTPD_SERVER) ||                                    \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE)
  state->own_identity.btc_priority1 = CONFIG_NETUTILS_PTPD_PRIORITY1;
  state->own_identity.btc_quality[0] = CONFIG_NETUTILS_PTPD_CLASS;
  state->own_identity.btc_quality[1] = CONFIG_NETUTILS_PTPD_ACCURACY;
  state->own_identity.btc_quality[2] = 0xff;
  state->own_identity.btc_quality[3] = 0xff;
  state->own_identity.btc_priority2 = CONFIG_NETUTILS_PTPD_PRIORITY2;
  memcpy(state->own_identity.btc_identity,
         state->own_identity.header.sourceidentity,
         sizeof(state->own_identity.btc_identity));
  state->own_identity.timesource = CONFIG_NETUTILS_PTPD_CLOCKSOURCE;
#else
  /* Statically configured as timereceiver: advertise worst priority. */
  state->own_identity.btc_priority1 = 255;
#endif

  s_state = state;

  ptpinfo("PTP daemon started in %s mode on port %d medium=%s\n",
          ptp_is_gptp(state) ? "gPTP" : "standard", args->port_index,
          args->medium == ptp_port_medium_eth_hwts ? "eth_hwts"
                                                   : "wifi_ftm");
  return OK;
}
#else
static int ptp_initialize_state(FAR struct ptp_state_s *state,
                                FAR const char *interface) {
  int ret;
  struct ifreq req;
  struct sockaddr_in bind_addr;

#ifdef CONFIG_NET_TIMESTAMP
  int arg;
#endif

  /* Create sockets */

  state->tx_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (state->tx_socket < 0) {
    ptperr("Failed to create tx socket: %d\n", errno);
    return ERROR;
  }

  state->event_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (state->event_socket < 0) {
    ptperr("Failed to create event socket: %d\n", errno);
    return ERROR;
  }

  state->info_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (state->info_socket < 0) {
    ptperr("Failed to create info socket: %d\n", errno);
    return ERROR;
  }

  /* Get address information of the specified interface for binding socket
   * Only supports IPv4 currently.
   */

  memset(&req, 0, sizeof(req));
  strncpy(req.ifr_name, interface, sizeof(req.ifr_name));

  if (ioctl(state->event_socket, SIOCGIFADDR, (unsigned long)&req) < 0) {
    ptperr("Failed to get IP address information for interface %s\n",
           interface);
    return ERROR;
  }

  state->interface_addr = *(struct sockaddr_in *)&req.ifr_ifru.ifru_addr;

  /* Get hardware address to initialize the identity field in header.
   * Clock identity is EUI-64, which we make from EUI-48.
   */

  if (ioctl(state->event_socket, SIOCGIFHWADDR, (unsigned long)&req) < 0) {
    ptperr("Failed to get HW address information for interface %s\n",
           interface);
    return ERROR;
  }

  state->own_identity.header.version = 2;
  state->own_identity.header.domain = CONFIG_NETUTILS_PTPD_DOMAIN;
  state->own_identity.header.sourceidentity[0] = req.ifr_hwaddr.sa_data[0];
  state->own_identity.header.sourceidentity[1] = req.ifr_hwaddr.sa_data[1];
  state->own_identity.header.sourceidentity[2] = req.ifr_hwaddr.sa_data[2];
  state->own_identity.header.sourceidentity[3] = 0xff;
  state->own_identity.header.sourceidentity[4] = 0xfe;
  state->own_identity.header.sourceidentity[5] = req.ifr_hwaddr.sa_data[3];
  state->own_identity.header.sourceidentity[6] = req.ifr_hwaddr.sa_data[4];
  state->own_identity.header.sourceidentity[7] = req.ifr_hwaddr.sa_data[5];
  state->own_identity.header.sourceportindex[0] = 0;
  state->own_identity.header.sourceportindex[1] = 1;
  state->own_identity.btc_priority1 = CONFIG_NETUTILS_PTPD_PRIORITY1;
  state->own_identity.btc_quality[0] = CONFIG_NETUTILS_PTPD_CLASS;
  state->own_identity.btc_quality[1] = CONFIG_NETUTILS_PTPD_ACCURACY;
  state->own_identity.btc_quality[2] = 0xff; /* No variance estimate */
  state->own_identity.btc_quality[3] = 0xff;
  state->own_identity.btc_priority2 = CONFIG_NETUTILS_PTPD_PRIORITY2;
  memcpy(state->own_identity.btc_identity,
         state->own_identity.header.sourceidentity,
         sizeof(state->own_identity.btc_identity));
  state->own_identity.timesource = CONFIG_NETUTILS_PTPD_CLOCKSOURCE;

  /* Subscribe to PTP multicast address */

  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);

  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_multicast);

  ret = ipmsfilter(&state->interface_addr.sin_addr, &bind_addr.sin_addr,
                   MCAST_INCLUDE);
  if (ret < 0) {
    ptperr("Failed to bind multicast address: %d\n", errno);
    return ERROR;
  }

  /* Bind socket for events */

  bind_addr.sin_port = HTONS(PTP_UDP_PORT_EVENT);
  ret = bind(state->event_socket, (struct sockaddr *)&bind_addr,
             sizeof(bind_addr));
  if (ret < 0) {
    ptperr("Failed to bind to udp port %d\n", bind_addr.sin_port);
    return ERROR;
  }

#ifdef CONFIG_NET_TIMESTAMP
  arg = 1;
  ret = setsockopt(state->event_socket, SOL_SOCKET, SO_TIMESTAMP, &arg,
                   sizeof(arg));

  if (ret < 0) {
    ptperr("Failed to enable SO_TIMESTAMP: %s\n", strerror(errno));

    /* PTPD can operate without, but with worse accuracy */
  }
#endif

  /* Bind socket for announcements */

  bind_addr.sin_port = HTONS(PTP_UDP_PORT_INFO);
  ret = bind(state->info_socket, (struct sockaddr *)&bind_addr,
             sizeof(bind_addr));
  if (ret < 0) {
    ptperr("Failed to bind to udp port %d\n", bind_addr.sin_port);
    return ERROR;
  }

  /* Bind TX socket to interface address (local addr cannot be multicast) */

  bind_addr.sin_addr = state->interface_addr.sin_addr;
  ret =
      bind(state->tx_socket, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
  if (ret < 0) {
    ptperr("Failed to bind tx to port %d\n", bind_addr.sin_port);
    return ERROR;
  }

  return OK;
}
#endif // ESP_PTP

/* Unsubscribe multicast and destroy sockets */

static int ptp_destroy_state(FAR struct ptp_state_s *state) {
#ifdef ESP_PTP
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
#else
  struct in_addr mcast_addr;

  mcast_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
  ipmsfilter(&state->interface_addr.sin_addr, &mcast_addr, MCAST_EXCLUDE);

  if (state->tx_socket > 0) {
    close(state->tx_socket);
    state->tx_socket = -1;
  }

  if (state->event_socket > 0) {
    close(state->event_socket);
    state->event_socket = -1;
  }

  if (state->info_socket > 0) {
    close(state->info_socket);
    state->info_socket = -1;
  }
#endif // ESP_PTP
  return OK;
}

#ifndef ESP_PTP
/* Re-subscribe multicast address.
 * This can become necessary if Ethernet interface gets reset or if external
 * IGMP-compliant Ethernet switch gets plugged in.
 */

static int ptp_check_multicast_status(FAR struct ptp_state_s *state) {
#if CONFIG_NETUTILS_PTPD_MULTICAST_TIMEOUT_MS > 0
  struct in_addr mcast_addr;
  struct timespec time_now;
  struct timespec delta;

  clock_gettime(CLOCK_MONOTONIC, &time_now);
  clock_timespec_subtract(&time_now, &state->port[0].last_received_multicast,
                          &delta);

  if (timespec_to_ms(&delta) > CONFIG_NETUTILS_PTPD_MULTICAST_TIMEOUT_MS) {
    /* Remove and re-add the multicast group */

    state->port[0].last_received_multicast = time_now;

    mcast_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
    ipmsfilter(&state->interface_addr.sin_addr, &mcast_addr, MCAST_EXCLUDE);

    return ipmsfilter(&state->interface_addr.sin_addr, &mcast_addr,
                      MCAST_INCLUDE);
  }

#else
  UNUSED(state);
#endif /* CONFIG_NETUTILS_PTPD_MULTICAST_TIMEOUT_MS */

  return OK;
}
#endif // !ESP_PTP

/* Track the source clockIdentity of a received Pdelay_Resp; if the count of
 * distinct responders crosses 2, latch pdelay_multi_responder.
 * Per profiles/avb_lite.md §2.2 cond 3 — spec-compliant AVB bridge presents
 * exactly one boundary-clock peer per port. */
static void ptp_record_pdelay_responder(FAR struct ptp_state_s *state,
                                        FAR const uint8_t *sourceidentity) {
  if (state->port[0].pdelay_multi_responder) {
    return;
  }
  for (unsigned i = 0; i < state->port[0].pdelay_resp_responder_count; i++) {
    if (memcmp(state->port[0].pdelay_resp_responders[i], sourceidentity, 8) ==
        0) {
      return; /* already tracked */
    }
  }
  if (state->port[0].pdelay_resp_responder_count <
      PTP_PDELAY_RESP_MAX_TRACKED) {
    memcpy(
        state->port[0]
            .pdelay_resp_responders[state->port[0].pdelay_resp_responder_count],
        sourceidentity, 8);
  }
  state->port[0].pdelay_resp_responder_count++;
  if (state->port[0].pdelay_resp_responder_count >= 2) {
    state->port[0].pdelay_multi_responder = true;
  }
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
    ptpinfo("Sent endpoint beacon, seq %ld\n",
            (long)ptp_get_sequence(&req.header));
  }
  return ret;
}

/* Append the AVB Lite Endpoint Declaration TLV to a Pdelay-class message
 * buffer at offset base_len. Returns the number of bytes added.
 * Per profiles/avb_lite.md §2.1.
 */
static size_t ptp_append_endpoint_decl_tlv(FAR uint8_t *msg_buf,
                                           size_t base_len) {
  FAR struct ptp_endpoint_decl_tlv_s *tlv =
      (FAR struct ptp_endpoint_decl_tlv_s *)(msg_buf + base_len);
  static const uint8_t orgid[3] = PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES;
  static const uint8_t orgsub[3] = PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES;

  tlv->type[0] = (PTP_TLV_TYPE_ORGANIZATION_EXTENSION >> 8) & 0xFF;
  tlv->type[1] = PTP_TLV_TYPE_ORGANIZATION_EXTENSION & 0xFF;
  tlv->length[0] = 0x00;
  tlv->length[1] = 0x07;
  memcpy(tlv->orgidentity, orgid, sizeof(orgid));
  memcpy(tlv->orgsubtype, orgsub, sizeof(orgsub));
  tlv->data = PTP_ENDPOINT_DECL_TLV_DATA;
  return sizeof(struct ptp_endpoint_decl_tlv_s);
}

/* Scan a Pdelay-class message for the AVB Lite Endpoint Declaration TLV.
 * msg_buf points at the message start, total_len is the on-wire length,
 * base_len is sizeof the standard message body (Pdelay_Req / Resp / Fup).
 * Returns true if the TLV is present and identifies the sender as an
 * AVB Lite endpoint. Per profiles/avb_lite.md §2.1.
 */
static bool ptp_msg_has_endpoint_decl_tlv(FAR const uint8_t *msg_buf,
                                          size_t total_len, size_t base_len) {
  static const uint8_t orgid[3] = PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES;
  static const uint8_t orgsub[3] = PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES;
  size_t offset = base_len;

  while (offset + 4 <= total_len) {
    uint16_t type = ((uint16_t)msg_buf[offset] << 8) | msg_buf[offset + 1];
    uint16_t len = ((uint16_t)msg_buf[offset + 2] << 8) | msg_buf[offset + 3];
    size_t body = offset + 4;
    if (body + len > total_len) {
      break; /* malformed / truncated */
    }
    if (type == PTP_TLV_TYPE_ORGANIZATION_EXTENSION && len >= 7 &&
        memcmp(msg_buf + body, orgid, 3) == 0 &&
        memcmp(msg_buf + body + 3, orgsub, 3) == 0 &&
        msg_buf[body + 6] == PTP_ENDPOINT_DECL_TLV_DATA) {
      return true;
    }
    offset = body + len;
  }
  return false;
}

/* Send PTP server announcement packet */

static int ptp_send_announce(FAR struct ptp_state_s *state) {
  struct ptp_announce_s msg;
#ifndef ESP_PTP
  struct sockaddr_in addr;
#endif // !ESP_PTP
  struct timespec ts;
  int ret;

#ifndef ESP_PTP
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
  addr.sin_port = HTONS(PTP_UDP_PORT_INFO);
#endif // !ESP_PTP

  memset(&msg, 0, sizeof(msg));
  msg = state->own_identity;
  msg.header.messagetype = PTP_MSGTYPE_ANNOUNCE;
  msg.header.messagelength[1] = sizeof(msg);
  msg.header.logmessageinterval =
      msec_to_log_period(CONFIG_NETUTILS_PTPD_ANNOUNCE_INTERVAL_MS);

  if (ptp_is_gptp(state)) {
    msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
    msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;   // gPTP required flag
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

#ifdef ESP_PTP
  ret = ptp_net_send(state, &msg, sizeof(msg), NULL);
#else
  ret = sendto(state->tx_socket, &msg, sizeof(msg), 0, (struct sockaddr *)&addr,
               sizeof(addr));
#endif // ESP_PTP

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
  } else {
    ptpinfo("Sent announce, seq %ld\n", (long)ptp_get_sequence(&msg.header));
  }

  return ret;
}

#ifdef ESP_PTP
/* Build the bytes that go into the §12.7 VendorSpecific information
 * element of an FTM/TM frame (or, in our case, the Beacon Vendor IE
 * the bridge uses as a workaround carrier — see ptp_beacon_ie.c and
 * the §12 deviation note in ESP-AVB-Bridge/project.md).
 *
 * Per §12.7: "the master state machine communicates an entire
 * Follow_Up message [i.e., including all the fields of the common
 * header (see 11.4.2 and 10.6.2), the preciseOriginTimestamp, and
 * all the fields of the Follow_Up information TLV (see 11.4.4)]
 * using this mechanism." The wire bytes match a normal 802.1AS
 * Follow_Up frame body (76 bytes): 34-byte common header + 10-byte
 * preciseOriginTimestamp + 32-byte FollowUpInformation TLV.
 *
 * Design choice — regenerate, don't forward.
 * The standard time-aware bridge model (§11.2.13) FORWARDS the
 * upstream Sync: keeps the upstream preciseOriginTimestamp,
 * accumulates residence time into correctionField. We instead
 * REGENERATE: preciseOriginTimestamp = ptpd_now() (our local
 * PTP-disciplined clock, which the wired-side PI servo has already
 * brought into the GM time domain). Equivalent under perfect lock —
 * both give the STA the same effective GM time — but simpler and
 * doesn't need separate residence-time accounting. The corollary
 * fields all carry zero by design:
 *   - correctionField = 0: the preciseOriginTimestamp is already
 *     GM-time (corrected by our local servo), no residence-time
 *     accumulation needed.
 *   - cumulativeScaledRateOffset = 0: our published time advances
 *     at GM rate (our local clock's drift is absorbed by the servo
 *     before we publish), so the rate offset is nominally zero.
 *   - gmTimeBaseIndicator / lastGmPhaseChange / scaledLastGmFreqChange
 *     = 0: ptpd doesn't yet track GM-reselection events. Add when
 *     BTCA reselection plumbing exists.
 *   - sequenceId = 0: the §12 model on Wi-Fi doesn't pair-match
 *     Sync/Follow_Up like the wired path (no separate Sync frame).
 *     STAs that want beacon-loss tracking can add a per-marshal
 *     counter.
 *
 * GM identity:
 *   The header's sourceClockIdentity is the upstream BTC's
 *   clockIdentity (state->selected_source.btc_identity) when we are
 *   synced upstream, so STAs see the wired GM in Hive; falls back
 *   to our own identity when we are the GM. */
static void ptp_marshal_follow_up_for_beacon_ie(
    FAR struct ptp_state_s *state, FAR struct ptp_follow_up_s *out) {
  memset(out, 0, sizeof(*out));

  /* Common header — start from cached own_identity template */
  out->header = state->own_identity.header;
  out->header.messagetype = PTP_MSGTYPE_FOLLOW_UP;
  out->header.controlfield = 2; /* IEEE 1588 Follow_Up controlfield */
  out->header.logmessageinterval =
      msec_to_log_period(CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MS);
  out->header.messagelength[0] = 0;
  out->header.messagelength[1] = sizeof(struct ptp_follow_up_s);
  out->header.flags[0] = 0; /* 2-step bit lives on Sync, cleared on Follow_Up */
  if (ptp_is_gptp(state)) {
    out->header.messagetype |= PTP_MSGTYPE_SDOID_GPTP;
    out->header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
  }

  /* sourceClockIdentity: upstream BTC if we are synced, else our own.
   * Without this override STAs see the bridge's MAC as the GM, which
   * Hive shows as `grandmaster_id` and the user-facing graph misses
   * the actual upstream BTC (e.g. MOTU 8D). */
  if (state->selected_source_valid) {
    memcpy(out->header.sourceidentity,
           state->selected_source.btc_identity,
           sizeof(out->header.sourceidentity));
  }

  /* preciseOriginTimestamp — current PTP-disciplined time */
  struct timespec ts;
  ptp_gettime(state, &ts);
  timespec_to_ptp_format(&ts, out->origintimestamp);

  /* FollowUpInformation TLV — §11.4.4.3. TLV header constants are
   * always populated; data fields are all zero by design (see
   * design-choice notes above). */
  struct ptp_info_tlv_s *tlv = (struct ptp_info_tlv_s *)out->informationtlv;
  tlv->type[1] = 3;                  /* Organization extension */
  tlv->length[1] = 0x1c;             /* 28 bytes of value */
  tlv->orgidentity[0] = 0x00;
  tlv->orgidentity[1] = 0x80;
  tlv->orgidentity[2] = 0xc2;        /* IEEE 802.1 OUI */
  tlv->orgsubtype[2] = 1;            /* FollowUp Information per §11.4.4.3 */
}
#endif /* ESP_PTP */

/* Send PTP server synchronization packet */

static int ptp_send_sync(FAR struct ptp_state_s *state) {
#ifndef ESP_PTP
  struct msghdr txhdr;
  struct iovec txiov;
#endif            // !ESP_PTP
  ptp_msgbuf msg; // using generic msgbuf to allow for larger follow-up size
#ifndef ESP_PTP
  struct sockaddr_in addr;
#endif // !ESP_PTP
  struct timespec ts;
#ifndef ESP_PTP
  uint8_t controlbuf[64];
#endif // !ESP_PTP
  int ret;

#ifndef ESP_PTP
  memset(&txhdr, 0, sizeof(txhdr));
  memset(&txiov, 0, sizeof(txiov));

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
  addr.sin_port = HTONS(PTP_UDP_PORT_EVENT);
#endif // !ESP_PTP

  memset(&msg, 0, sizeof(msg));
  msg.header = state->own_identity.header;
  msg.header.messagetype = PTP_MSGTYPE_SYNC;
  msg.header.messagelength[1] = sizeof(struct ptp_sync_s);
  msg.header.logmessageinterval =
      msec_to_log_period(CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MS);

#if defined(CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC) ||                              \
    defined(                                                                   \
        CONFIG_NETUTILS_PTPD_GPTP_PROFILE) // gPTP always uses two-step sync
  msg.header.flags[0] = PTP_FLAGS0_TWOSTEP;
#endif
  if (ptp_is_gptp(state)) {
    msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP; // gPTP profile message
    msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;   // gPTP required flag
  }

#ifndef ESP_PTP
  txhdr.msg_name = &addr;
  txhdr.msg_namelen = sizeof(addr);
  txhdr.msg_iov = &txiov;
  txhdr.msg_iovlen = 1;
  txhdr.msg_control = controlbuf;
  txhdr.msg_controllen = sizeof(controlbuf);
  txiov.iov_base = &msg;
  txiov.iov_len = sizeof(msg);
#endif //! ESP_PTP

  /* Timestamp and send the sync message */

  ptp_increment_sequence(&state->sync_seq, &msg.header);
  ptp_gettime(state, &ts);
  timespec_to_ptp_format(&ts, msg.sync.origintimestamp);

#ifdef ESP_PTP
  ret = ptp_net_send(state, &msg, sizeof(struct ptp_sync_s), &ts);
#else
  ret = sendmsg(state->tx_socket, &txhdr, 0);
#endif // ESP_PTP
  if (ret < 0) {
    ptperr("sendmsg for sync message failed: %d\n", errno);
    return ret;
  }

#if defined(CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC) ||                              \
    defined(                                                                   \
        CONFIG_NETUTILS_PTPD_GPTP_PROFILE) // gPTP always uses two-step sync

  /* Send the follow up message */

#ifndef ESP_PTP
  /* Get timestamp after send completes and send follow-up message
   *
   * TODO: Implement SO_TIMESTAMPING and use the actual tx timestamp here.
   */

  ptp_gettime(state, &ts);
#endif // !ESP_PTP
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

#ifndef ESP_PTP
  addr.sin_port = HTONS(PTP_UDP_PORT_INFO);

  ret = sendto(state->tx_socket, &msg, sizeof(msg), 0, (struct sockaddr *)&addr,
               sizeof(addr));
#else
  ret = ptp_net_send(state, &msg, sizeof(struct ptp_follow_up_s), NULL);
#endif // !ESP_PTP
  if (ret < 0) {
    ptperr("sendto for follow-up message failed: %d\n", errno);
    return ret;
  }

  ptpinfo("Sent sync + follow-up, seq %ld\n",
          (long)ptp_get_sequence(&msg.header));
#else
  ptpinfo("Sent sync, seq %ld\n", (long)ptp_get_sequence(&msg.header));
#endif /* CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC */

  return OK;
}

/* Send delay request packet to selected source */

static int ptp_send_delay_req(FAR struct ptp_state_s *state) {
  ptp_msgbuf req;
#ifndef ESP_PTP
  struct sockaddr_in addr;
#endif // !ESP_PTP
  int ret;
  size_t req_len;

#ifndef ESP_PTP
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
  addr.sin_port = HTONS(PTP_UDP_PORT_EVENT);
#endif // !ESP_PTP

  memset(&req, 0, sizeof(req));
  req.header = state->own_identity.header;
  req.header.logmessageinterval =
      msec_to_log_period(state->port[0].delayreq_interval_ms);

  if (ptp_is_gptp(state)) {
    req.header.messagetype = PTP_MSGTYPE_PDELAY_REQ | PTP_MSGTYPE_SDOID_GPTP;
    req.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
    req.header.controlfield = 5;
    req_len = sizeof(struct ptp_pdelay_req_s);
    /* Append AVB Lite Endpoint Declaration TLV (profiles/avb_lite.md §2.1) */
    req_len += ptp_append_endpoint_decl_tlv(req.raw, req_len);
    /* Reset per-cycle responder set so the §2.2 cond 3 (cardinality) check
     * sees only the responders for this single Pdelay_Req. */
    state->port[0].pdelay_resp_responder_count = 0;
    memset(state->port[0].pdelay_resp_responders, 0,
           sizeof(state->port[0].pdelay_resp_responders));
  } else {
    req.header.messagetype = PTP_MSGTYPE_DELAY_REQ;
    ptp_gettime(state, &state->port[0].delayreq_time);
    timespec_to_ptp_format(&state->port[0].delayreq_time,
                           req.delay_req.origintimestamp);
    req_len = sizeof(struct ptp_delay_req_s);
  }
  req.header.messagelength[1] = req_len;

  ptp_increment_sequence(&state->delay_req_seq, &req.header);

#ifdef ESP_PTP
  ret = ptp_net_send(state, &req, req_len, &state->port[0].delayreq_time);
#else
  ret = sendto(state->tx_socket, &req, req_len, 0, (FAR struct sockaddr *)&addr,
               sizeof(addr));
#endif // ESP_PTP

#ifndef ESP_PTP
  /* Get timestamp after send completes.
   * TODO: Implement SO_TIMESTAMPING and use the actual tx timestamp here.
   */

  ptp_gettime(state, &state->port[0].delayreq_time);
#endif // !ESP_PTP

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
  } else {
    clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_transmitted_delayreq);
    if (ptp_is_gptp(state) && !state->gptp_fallback_done) {
      /* For AVB Lite fallback condition 2 (profiles/avb_lite.md §2.2). */
      state->port[0].pdelay_req_attempts_unanswered++;
    }
    ptpinfo("Sent delay req, seq %ld\n", (long)ptp_get_sequence(&req.header));
  }

  return ret;
}

static void ptp_check_profile_fallback(FAR struct ptp_state_s *state) {
  if (!ptp_is_gptp(state) || state->gptp_fallback_done) {
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
    state->ptp_profile = ptp_profile_standard;
    ptp_reset_for_profile(state);
    return;
  }

  /* AVB Lite fallback condition 2 (profiles/avb_lite.md §2.2):
   * three consecutive Pdelay_Req attempts with no Pdelay_Resp received. */

  if (state->port[0].pdelay_req_attempts_unanswered >= 3) {
    ptpwarn(
        "No Pdelay_Resp after %u attempts; switching to standard PTP mode\n",
        state->port[0].pdelay_req_attempts_unanswered);
    state->gptp_fallback_done = true;
    state->ptp_profile = ptp_profile_standard;
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
    state->ptp_profile = ptp_profile_standard;
    ptp_reset_for_profile(state);
  }
}

/* Check if we need to send packets */

static int ptp_periodic_send(FAR struct ptp_state_s *state) {
#if defined(CONFIG_NETUTILS_PTPD_SERVER) ||                                    \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE)
  /* If there is no better timetransmitter clock on the network,
   * act as the reference source and send server packets. */

  /* The "I am GM" path emits Announce + Sync via port[0]'s L2TAP socket
   * (ptp_send_announce / ptp_send_sync both write to
   * state->port[0].ptp_socket). That socket only exists on an eth_hwts
   * port. Skip this whole block if port[0] isn't socket-backed —
   * otherwise sendto returns EBADF on every cadence tick. wifi_ftm
   * ports advertise time via the §12.7 Vendor IE through the
   * sync_egress_cb loop further down, which has its own gating. */
  if (!state->selected_source_valid &&
      state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0) {
    struct timespec time_now;
    struct timespec delta;

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now,
                            &state->port[0].last_transmitted_announce, &delta);
    if (timespec_to_ms(&delta) > CONFIG_NETUTILS_PTPD_ANNOUNCE_INTERVAL_MS) {
      state->port[0].last_transmitted_announce = time_now;
      ptp_send_announce(state);
    }

    clock_timespec_subtract(&time_now, &state->port[0].last_transmitted_sync,
                            &delta);
    if (timespec_to_ms(&delta) > CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MS) {
      state->port[0].last_transmitted_sync = time_now;
      ptp_send_sync(state);
    }
  }
#endif /* CONFIG_NETUTILS_PTPD_SERVER */

#ifdef ESP_PTP
  /* Sync emission on wifi_ftm ports. Independent of selected_source_valid:
   * on a transparent time-aware bridge the wifi_ftm AP port republishes
   * upstream Sync at the configured cadence; when we are the GM the same
   * cadence applies and the payload reflects our own clock. Skipped when
   * no egress callback is registered.
   *
   * Payload is the §12.7 VendorSpecific IE "FollowUpInformation" — which
   * per §12.7 is the entire Follow_Up message (header + preciseOrigin-
   * Timestamp + FollowUpInformation TLV, 76 bytes), packed in exactly
   * the same format as the wired Follow_Up frame. */
  for (int p = 0; p < CONFIG_ESP_PTP_NUM_PORTS; p++) {
    struct ptp_port_s *port = &state->port[p];
    if (!port->enabled) continue;
    if (port->medium != ptp_port_medium_wifi_ftm) continue;
    if (port->sync_egress_cb == NULL) continue;

    struct timespec time_now, delta;
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now, &port->last_transmitted_sync, &delta);
    if (timespec_to_ms(&delta) <= CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MS) {
      continue;
    }
    port->last_transmitted_sync = time_now;

    struct ptp_follow_up_s fu;
    ptp_marshal_follow_up_for_beacon_ie(state, &fu);
    port->sync_egress_cb(p, (const uint8_t *)&fu, sizeof(fu),
                         port->sync_egress_ctx);
  }
#endif /* ESP_PTP */

#if defined(CONFIG_NETUTILS_PTPD_SEND_DELAYREQ) ||                             \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE)
  /* Delay_Req / Pdelay_Req TX also writes through port[0]'s L2TAP
   * socket. Skip on wifi_ftm-only deployments — peer-delay there is
   * out-of-band via FTM + ptpd_inject_peer_delay, not on-wire. */
  if ((ptp_is_gptp(state) ||
       (state->selected_source_valid && state->port[0].can_send_delayreq)) &&
      state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0) {
    struct timespec time_now;
    struct timespec delta;

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    clock_timespec_subtract(&time_now,
                            &state->port[0].last_transmitted_delayreq, &delta);
    if (timespec_to_ms(&delta) > state->port[0].next_delayreq_interval_ms) {
      /* Skip the first tx (last_transmitted_delayreq == 0) so startup
       * isn't counted as "late". */
      if (state->port[0].last_transmitted_delayreq.tv_sec != 0) {
        int64_t late_ms = (int64_t)timespec_to_ms(&delta) -
                          (int64_t)state->port[0].next_delayreq_interval_ms;
        ptpd_lateness_record_tx(late_ms * 1000LL);
      }
      ptp_send_delay_req(state);
    }
  }
#endif // CONFIG_NETUTILS_PTPD_SEND_DELAYREQ ||
       // CONFIG_NETUTILS_PTPD_GPTP_PROFILE

  /* Post-fallback endpoint beacon (profiles/avb_lite.md §2.3): once we've
   * fallen back to standard PTP, periodically emit a Pdelay_Req-with-TLV
   * to the gPTP bridge-group MAC so peers still in gPTP can detect us and
   * follow us into the standard profile. Cadence is 3 s, matching the
   * §2.2 evaluation window so any peer's window is guaranteed to overlap
   * at least one beacon. */

  /* Endpoint-fallback beacon — only on eth_hwts with a socket. */
  if (state->gptp_fallback_done && state->ptp_profile == ptp_profile_standard &&
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
                                FAR struct ptp_announce_s *msg) {
  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_announce);

  if (is_better_clock(msg, &state->own_identity)) {
    if (!state->selected_source_valid ||
        is_better_clock(msg, &state->selected_source)) {
      ptpinfo("Switching to better PTP time source\n");

      state->selected_source = *msg;
      state->port[0].last_received_sync = state->port[0].last_received_announce;
      state->port[0].path_delay_avgcount = 0;
      state->port[0].path_delay_ns = 0;
      state->port[0].peer_delay_avgcount = 0;
      state->port[0].peer_delay_ns = 0;
      state->correction_ns = 0;
      state->port[0].delayreq_time.tv_sec = 0;
    }
  }

  return OK;
}

#ifdef ESP_PTP
static void ptp_lock_local_clock_freq(FAR struct ptp_state_s *state,
                                      FAR struct timespec *remote_timestamp,
                                      FAR struct timespec *local_timestamp) {
  // Compute how off we are against the timetransmitter
  int64_t offset_ns = timespec_delta_ns(remote_timestamp, local_timestamp);
  if (ptp_is_gptp(state)) {
    offset_ns += state->port[0].peer_delay_ns + state->correction_ns;
  } else {
    offset_ns += state->port[0].path_delay_ns;
  }
  // TODO add offset filter

  // Execute PI controller to elimitate the offset
  // compute I component
  state->offset_pi.drift_acc += offset_ns / state->offset_pi.ki;
  // clamp the accumulator to ADJ_FREQ_MAX for sanity
  if (state->offset_pi.drift_acc > ADJ_FREQ_MAX) {
    state->offset_pi.drift_acc = ADJ_FREQ_MAX;
  } else if (state->offset_pi.drift_acc < -ADJ_FREQ_MAX) {
    state->offset_pi.drift_acc = -ADJ_FREQ_MAX;
  }
  // compute P component and the whole controller
  int32_t adj = offset_ns / state->offset_pi.kp + state->offset_pi.drift_acc;

  // Compute the tick-count difference between local (timereceiver)
  // and remote (timetransmitter) over the sync period. Used to lock
  // frequency; doesn't catch up the offset by itself, hence `adj`.
  int64_t remote_time_ns = timespec_to_ns(remote_timestamp);
  int64_t local_time_ns = timespec_to_ns(local_timestamp);
  int64_t remote_delta_ns = remote_time_ns - state->remote_time_ns_prev;
  int64_t local_delta_ns = local_time_ns - state->local_time_ns_prev;
  // Clock tick difference between timetransmitter and timereceiver
  int64_t tick_diff = remote_delta_ns - local_delta_ns;

  // Scale the local frequency to lock with the timetransmitter's,
  // and try to catch up the offset
  double freq_scale = ((double)(remote_delta_ns /*+ tick_diff*/ + adj)) /
                      (double)local_delta_ns;
  long freq_ppb = (long)((freq_scale - 1.0) * 1e9);
  if (s_use_sw_clock) {
    /* SW backend caps at +/- 1e8 ppb; let it clamp by returning -1. */
    ptp_clock_sw_adjtime_rate((int32_t)freq_ppb);
  } else {
    struct timex tx = {
        .modes = ADJ_FREQUENCY,
        .freq = freq_ppb,
    };
    clock_adjtime(PTPD_CLOCK_ID, &tx);
  }

  state->remote_time_ns_prev = remote_time_ns;
  state->local_time_ns_prev = local_time_ns;

  ptpinfo("remote_delta_ns %lli, local_delta_ns %lli, tick_diff %lli\n",
          remote_delta_ns, local_delta_ns, tick_diff);
  ptpinfo("offset_ns %lli, adj %li, drift_acc %li\n", offset_ns, adj,
          state->offset_pi.drift_acc);

  // Get the path delay only when clock is stable enough. If we were
  // in the process of speeding/slowing the local clock, we'd get an
  // incorrect delay measurement
  int64_t diff = llabs(offset_ns) - llabs(state->last_offset_ns);
  static int cnt = 0;
  if ((ptp_is_gptp(state) &&
       llabs(diff) < CONFIG_NETUTILS_PTPD_PEER_DELAY_STABILITY_NS) ||
      (!ptp_is_gptp(state) &&
       llabs(diff) < CONFIG_NETUTILS_PTPD_PATH_DELAY_STABILITY_NS)) {
    if (cnt <= 3)
      cnt++;
  } else {
    cnt = 0;
  }
  if (cnt > 3) {
    ptpinfo("clock is stablized\n");
    state->port[0].can_send_delayreq = true;
  } else {
    ptpinfo("clock is still unstable\n");
  }
  state->last_offset_ns = offset_ns;
}

static void ptp_clean_after_step(FAR struct ptp_state_s *state) {
  state->remote_time_ns_prev = 0;
  state->local_time_ns_prev = 0;

  state->offset_pi.drift_acc = 0;
  state->last_offset_ns = 0;
}
#endif // ESP_PTP

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
      CONFIG_NETUTILS_PTPD_SETTIME_THRESHOLD_MS * (int64_t)NSEC_PER_MSEC;

  ptpinfo("Local time: %lld.%09ld, remote time %lld.%09ld\n",
          (long long)local_timestamp->tv_sec, (long)local_timestamp->tv_nsec,
          (long long)remote_timestamp->tv_sec, (long)remote_timestamp->tv_nsec);

  delta_ns = timespec_delta_ns(remote_timestamp, local_timestamp);
  if (ptp_is_gptp(state)) {
    delta_ns += state->port[0].peer_delay_ns;
  } else {
    delta_ns += state->port[0].path_delay_ns;
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

#ifdef ESP_PTP
    ptp_clean_after_step(state);
#endif // ESP_PTP

    if (ret == OK) {
      ptpinfo("Jumped to timestamp %lld.%09ld s\n", (long long)new_time.tv_sec,
              (long)new_time.tv_nsec);
    } else {
      ptperr("ptp_settime() failed: %d\n", errno);
    }
  } else {
#ifdef ESP_PTP
    ptp_lock_local_clock_freq(state, remote_timestamp, local_timestamp);
#else
    /* Track drift rate based on two consecutive measurements and
     * the adjustment that was made previously.
     */

    int64_t drift_ppb;
    struct timespec interval;
    int interval_ms;
    int max_avg_period_ms;
    int64_t adjustment_ns;

    clock_timespec_subtract(local_timestamp, &state->last_delta_timestamp,
                            &interval);
    interval_ms = timespec_to_ms(&interval);

    if (interval_ms > 0 && interval_ms < CONFIG_NETUTILS_PTPD_TIMEOUT_MS) {
      drift_ppb =
          (delta_ns - state->last_delta_ns) * MSEC_PER_SEC / interval_ms;
    } else {
      ptpwarn("Measurement interval out of range: %d ms\n", interval_ms);
      drift_ppb = 0;
      interval_ms = 1;
    }

    /* Account for the adjustment previously made */

    drift_ppb +=
        state->last_adjtime_ns * MSEC_PER_SEC / CONFIG_CLOCK_ADJTIME_PERIOD_MS;

    if (drift_ppb > CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM * 1000 ||
        drift_ppb < -CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM * 1000) {
      ptpwarn("Drift estimate out of range: %lld\n", (long long)drift_ppb);
      drift_ppb = state->drift_ppb;
    }

    /* Take direct average of drift estimate for first measurements,
     * after that update the exponential sliding average.
     * Measurements are weighted according to the interval, because
     * drift estimate is more accurate over longer timespan.
     */

    state->drift_avg_total_ms += interval_ms;
    max_avg_period_ms = CONFIG_NETUTILS_PTPD_DRIFT_AVERAGE_S * MSEC_PER_SEC;
    if (state->drift_avg_total_ms > max_avg_period_ms) {
      state->drift_avg_total_ms = max_avg_period_ms;
    }

    state->drift_ppb += (drift_ppb - state->drift_ppb) * interval_ms /
                        state->drift_avg_total_ms;

    /* Compute the value we need to give to adjtime() to match the
     * drift rate.
     */

    adjustment_ns =
        state->drift_ppb * CONFIG_CLOCK_ADJTIME_PERIOD_MS / MSEC_PER_SEC;

    /* Drift estimation ensures local clock runs at same rate as remote.
     *
     * Adding the current clock offset to adjustment brings the clocks
     * to match. To avoid individual outliers from causing jitter, we
     * take the larger signed value of two previous deltas. This is based
     * on the logic that packets can get delayed in transit, but do not
     * travel backwards in time.
     *
     * Clock offset is applied over ADJTIME_PERIOD. If there is significant
     * noise in measurements, increasing ADJTIME_PERIOD will reduce its
     * effect on the local clock run rate.
     */

    if (state->last_delta_ns > delta_ns) {
      adjustment_ns += state->last_delta_ns;
    } else {
      adjustment_ns += delta_ns;
    }

    /* Apply adjustment and store information for next time */

    state->last_delta_ns = delta_ns;
    state->last_delta_timestamp = *local_timestamp;
    state->last_adjtime_ns = adjustment_ns;

    ptpinfo("Delta: %+lld ns, adjustment %+lld ns, drift rate %+lld ppb\n",
            (long long)delta_ns, (long long)state->last_adjtime_ns,
            (long long)state->drift_ppb);

    ret = ptp_adjtime(state, adjustment_ns);

    if (ret != OK) {
      ptperr("ptp_adjtime() failed: %d\n", errno);
    }

    /* Check if clock is stable enough for sending delay requests */

    if (absdelta_ns < CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS) {
      state->port[0].can_send_delayreq = true;
    }
#endif // ESP_PTP
  }

  return ret;
}

/* Process received PTP sync packet */

static int ptp_process_sync(FAR struct ptp_state_s *state,
                            FAR struct ptp_sync_s *msg) {
  struct timespec remote_time;

  if (memcmp(msg->header.sourceidentity,
             state->selected_source.header.sourceidentity,
             sizeof(msg->header.sourceidentity)) != 0) {
    /* This packet wasn't from the currently selected source */
#ifdef ESP_PTP
    ESP_LOGD(TAG, "This packet wasn't from the currently selected source");
#endif // ESP_PTP
    return OK;
  }

  /* Update timeout tracking */

  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_sync);

  if (msg->header.flags[0] & PTP_FLAGS0_TWOSTEP) {
    /* We need to wait for a follow-up packet before setting the clock. */

    state->port[0].twostep_rxtime = state->port[0].rxtime;
    state->port[0].twostep_packet = *msg;
    ptpinfo("Waiting for follow-up\n");
    return OK;
  }

  /* Update local clock */

  ptp_format_to_timespec(msg->origintimestamp, &remote_time);
  return ptp_update_local_clock(state, &remote_time, &state->port[0].rxtime);
}

static int ptp_process_followup(FAR struct ptp_state_s *state,
                                FAR struct ptp_follow_up_s *msg) {
  struct timespec remote_time;

  if (memcmp(msg->header.sourceidentity,
             state->port[0].twostep_packet.header.sourceidentity,
             sizeof(msg->header.sourceidentity)) != 0) {
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

  /* Update local clock based on the remote timestamp we received now
   * and the local timestamp of when the sync packet was received.
   * For gPTP, we can also examine the information TLV for other changes
   */

  ptp_format_to_timespec(msg->origintimestamp, &remote_time);
  return ptp_update_local_clock(state, &remote_time,
                                &state->port[0].twostep_rxtime);
}

static int ptp_process_delay_req(FAR struct ptp_state_s *state,
                                 FAR struct ptp_delay_req_s *req) {
  ptp_msgbuf
      resp; // using generic message buffer to allow for larger follow-up size
  struct timespec ts;
#ifndef ESP_PTP
  struct sockaddr_in addr;
#endif // !ESP_PTP
  int ret;

  if (!ptp_is_gptp(state) && state->selected_source_valid) {
    /* We are operating as a standard PTP client, ignore delay requests */

    return OK;
  }

#ifndef ESP_PTP
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = HTONL(PTP_MULTICAST_ADDR);
  addr.sin_port = HTONS(PTP_UDP_PORT_INFO);
#endif // !ESP_PTP

  memset(&resp, 0, sizeof(resp));
  resp.header = state->own_identity.header;
  resp.header.messagetype =
      ptp_is_gptp(state) ? PTP_MSGTYPE_PDELAY_RESP : PTP_MSGTYPE_DELAY_RESP;
  size_t resp_len = sizeof(struct ptp_delay_resp_s);

#if defined(CONFIG_NETUTILS_PTPD_TWOSTEP_SYNC) ||                              \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE)
  resp.header.flags[0] = PTP_FLAGS0_TWOSTEP;
#endif

  if (ptp_is_gptp(state)) {
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
  // this is typically ignored in a response, except in the case of gPTP
  resp.header.logmessageinterval =
      msec_to_log_period(CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS);

  /* Append AVB Lite Endpoint Declaration TLV to gPTP Pdelay_Resp
   * (profiles/avb_lite.md §2.1) */
  if (ptp_is_gptp(state)) {
    resp_len += ptp_append_endpoint_decl_tlv(resp.raw, resp_len);
  }
  resp.header.messagelength[1] = resp_len;

  /* Send the response message */

#ifdef ESP_PTP
  ret = ptp_net_send(state, &resp, resp_len, &ts);
#else
  ret = sendto(state->tx_socket, &resp, sizeof(resp), 0,
               (FAR struct sockaddr *)&addr, sizeof(addr));
#endif // ESP_PTP

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
    fup_len += ptp_append_endpoint_decl_tlv(resp.raw, fup_len);
    resp.header.messagelength[1] = fup_len;
    resp.header.flags[0] = 0; // Reset 2-step flag

    /* Send the response follow-up message, currently only for ESP as it
     * requires hw timestamp data */

#ifdef ESP_PTP
    ret = ptp_net_send(state, &resp, fup_len, NULL);
#endif // ESP_PTP

    if (ret < 0) {
      ptperr("sendto for delay response follow-up message failed: %d\n", errno);
      return ret;
    }
    ptpinfo("Sent response + response follow-up, seq %ld\n",
            (long)ptp_get_sequence(&resp.header));
  } else {
    ptpinfo("Sent delay resp, seq %ld\n", (long)ptp_get_sequence(&req->header));
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
    /* gPTP peer delay responses are valid from any peer */
    if (memcmp(msg->reqidentity, state->own_identity.header.sourceidentity,
               sizeof(msg->reqidentity)) != 0) {
      return OK; /* This packet wasn't for us */
    }
  } else {
    if (!state->selected_source_valid ||
        memcmp(msg->header.sourceidentity,
               state->selected_source.header.sourceidentity,
               sizeof(msg->header.sourceidentity)) != 0 ||
        memcmp(msg->reqidentity, state->own_identity.header.sourceidentity,
               sizeof(msg->reqidentity)) != 0) {
      return OK; /* This packet wasn't for us */
    }
  }

  if (ptp_is_gptp(state)) {
    /* AVB Lite §2.2 cond 2 reset — receiving a Pdelay_Resp clears the
     * unanswered-attempt counter. We deliberately do NOT latch
     * gptp_fallback_done here: §2.2 cond 1 (Endpoint Declaration TLV) must
     * still be evaluated in subsequent ptp_check_profile_fallback calls. */
    state->port[0].pdelay_req_attempts_unanswered = 0;
  }

  sequence = ptp_get_sequence(&msg->header);

  if (sequence != state->delay_req_seq) {
    ptpwarn("Ignoring out-of-sequence delay resp (%d vs. expected %d)\n",
            (int)sequence, (int)state->delay_req_seq);
    return OK;
  }

  if (ptp_is_gptp(state)) {
    /* We need to wait for a resp follow-up to calc peer delay. */

    state->port[0].twostep_delay_resp_rxtime = state->port[0].rxtime;
    state->port[0].twostep_delay_resp_packet = *msg;
    ptpinfo("Waiting for delay response follow-up\n");
  } else {
    /* Path delay is calculated as the average between delta for sync
     * message and delta for delay req message.
     * (IEEE-1588 section 11.3: Delay request-response mechanism)
     */

    ptp_format_to_timespec(msg->receivetimestamp, &remote_rxtime);
    path_delay =
        timespec_delta_ns(&remote_rxtime, &state->port[0].delayreq_time);
    sync_delay = state->port[0].path_delay_ns - state->last_delta_ns;
    path_delay = (path_delay + sync_delay) / 2;

    if (path_delay >= 0 &&
        path_delay < CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS) {
      if (state->port[0].path_delay_avgcount <
          CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT) {
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

  /* Calculate interval until next packet */
  if (msg->header.logmessageinterval <= 12) {
    state->port[0].delayreq_interval_ms =
        log_period_to_msec(msg->header.logmessageinterval);
  } else {
    state->port[0].delayreq_interval_ms =
        CONFIG_NETUTILS_PTPD_DELAYREQ_INTERVAL_MS;
  }

  /* Randomize delay for next interval */

  state->port[0].next_delayreq_interval_ms =
      rand_delayreq_interval(state->port[0].delayreq_interval_ms);
  ptpinfo("Randomized delay req interval: %d ms\n",
          state->port[0].next_delayreq_interval_ms);
  return OK;
}

static int
ptp_process_delay_resp_follow_up(FAR struct ptp_state_s *state,
                                 FAR struct ptp_delay_resp_follow_up_s *msg) {
  if (!ptp_is_gptp(state)) {
    return OK;
  }

  int64_t peer_delay_roundtrip;
  int64_t peer_delay_reflection;
  int64_t peer_delay;
  struct timespec remote_txtime;
  struct timespec remote_rxtime;

  if (memcmp(msg->reqidentity, state->own_identity.header.sourceidentity,
             sizeof(msg->reqidentity)) != 0)

  {
    return OK; /* This packet wasn't for us */
  }

  if (ptp_get_sequence(&msg->header) !=
      ptp_get_sequence(&state->port[0].twostep_delay_resp_packet.header)) {
    ptpwarn("PTP delay response follow-up packet sequence %ld does not "
            "match initial sync packet sequence %ld, ignoring\n",
            (long)ptp_get_sequence(&msg->header),
            (long)ptp_get_sequence(
                &state->port[0].twostep_delay_resp_packet.header));
    return OK;
  }

  /* In gPTP (802.1AS), delay is measured between peers, not
  * between the server and the client. It is calculated as follows:

     Peer A                          Peer B
     |                                    |
     |          Peer delay_req            |
  t1 |----------------------------------->| t2
     |                                    |
     |        Peer delay_resp (t2)        |
  t4 |<-----------------------------------| t3
     |                                    |
     |   Peer delay_resp_follow_up (t3)   |
     |<-----------------------------------|

    Peer A calculates peer_delay = ((t4 - t1) - (t3 - t2))/2
  */

  /* Calculate peer delay */

  peer_delay_roundtrip = timespec_delta_ns(
      &state->port[0].twostep_delay_resp_rxtime, &state->port[0].delayreq_time);
  ptp_format_to_timespec(
      state->port[0].twostep_delay_resp_packet.receivetimestamp,
      &remote_rxtime);
  ptp_format_to_timespec(msg->origintimestamp, &remote_txtime);
  peer_delay_reflection = timespec_delta_ns(&remote_txtime, &remote_rxtime);
  peer_delay = (peer_delay_roundtrip - peer_delay_reflection) / 2;

  if (peer_delay >= 0 && peer_delay < CONFIG_NETUTILS_PTPD_MAX_PEER_DELAY_NS) {
    if (state->port[0].peer_delay_avgcount <
        CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT) {
      state->port[0].peer_delay_avgcount++;
    }

    state->port[0].peer_delay_ns +=
        (peer_delay - state->port[0].peer_delay_ns) /
        state->port[0].peer_delay_avgcount;

    ptpinfo("Peer delay: %ld ns (avg: %ld ns)\n", (long)peer_delay,
            (long)state->port[0].peer_delay_ns);
  } else {
    ptpwarn("Peer delay out of range: %lld ns\n", (long long)peer_delay);
  }

  /* Update correction field */

  double correction_ns;
  correction_ns = get_correction_ns(msg->header.correction);
  memcpy(&state->correction_ns, &correction_ns, sizeof(correction_ns));

  return OK;
}

/* Determine received packet type and process it */

static int ptp_process_rx_packet(FAR struct ptp_state_s *state,
                                 ssize_t length) {
  if (length < sizeof(struct ptp_header_s)) {
    ptpwarn("Ignoring invalid PTP packet, length only %d bytes\n", (int)length);
    return OK;
  }

  if (state->port[0].rxbuf.header.domain != CONFIG_NETUTILS_PTPD_DOMAIN) {
    /* Part of different clock domain, ignore */

    return OK;
  }

  bool msg_is_gptp =
      (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_SDOID_GPTP) != 0;
  if (msg_is_gptp != ptp_is_gptp(state)) {
    return OK;
  }

  clock_gettime(CLOCK_MONOTONIC, &state->port[0].last_received_multicast);

  /* Rout the packet to the appropriate handler */

  switch (state->port[0].rxbuf.header.messagetype & PTP_MSGTYPE_MASK) {
#if defined(CONFIG_NETUTILS_PTPD_CLIENT) ||                                    \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE) // gPTP always acts as a client
  case PTP_MSGTYPE_ANNOUNCE:
    s_ptpd_rx_announce++;
    ptpinfo("Got announce packet, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    return ptp_process_announce(state, &state->port[0].rxbuf.announce);

  case PTP_MSGTYPE_SYNC:
    ptpd_lateness_record_rx_sync();
    ptpinfo("Got sync packet, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (!state->selected_source_valid) {
      return OK;
    } // ignore if operating as a server in gPTP profile
    return ptp_process_sync(state, &state->port[0].rxbuf.sync);

  case PTP_MSGTYPE_FOLLOW_UP:
    s_ptpd_rx_followup++;
    ptpinfo("Got follow-up packet, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (!state->selected_source_valid) {
      return OK;
    } // ignore if operating as a server in gPTP profile
    return ptp_process_followup(state, &state->port[0].rxbuf.follow_up);

  case PTP_MSGTYPE_DELAY_RESP:
  case PTP_MSGTYPE_PDELAY_RESP:
    s_ptpd_rx_pdelay_resp++;
    ptpinfo("Got delay-resp, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(state->port[0].rxbuf.raw, length,
                                      sizeof(struct ptp_delay_resp_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    if (!state->gptp_fallback_done) {
      ptp_record_pdelay_responder(state,
                                  state->port[0].rxbuf.header.sourceidentity);
    }
    return ptp_process_delay_resp(state, &state->port[0].rxbuf.delay_resp);
#endif

#if defined(CONFIG_NETUTILS_PTPD_SERVER) ||                                    \
    defined(CONFIG_NETUTILS_PTPD_GPTP_PROFILE) // gPTP always responds to delay
                                               // requests
  case PTP_MSGTYPE_DELAY_REQ:
  case PTP_MSGTYPE_PDELAY_REQ:
    s_ptpd_rx_pdelay_req++;
    ptpinfo("Got delay req, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(state->port[0].rxbuf.raw, length,
                                      sizeof(struct ptp_pdelay_req_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    return ptp_process_delay_req(state, &state->port[0].rxbuf.delay_req);
#endif

  case PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP:
    s_ptpd_rx_pdelay_fup++;
    ptpinfo("Got peer delay resp follow-up, seq %ld\n",
            (long)ptp_get_sequence(&state->port[0].rxbuf.header));
    if (ptp_msg_has_endpoint_decl_tlv(
            state->port[0].rxbuf.raw, length,
            sizeof(struct ptp_delay_resp_follow_up_s))) {
      state->port[0].peer_is_endpoint = true;
    }
    return ptp_process_delay_resp_follow_up(
        state, &state->port[0].rxbuf.delay_resp_follow_up);
  default:
    ptpinfo("Ignoring unknown PTP packet type: 0x%02x\n",
            state->port[0].rxbuf.header.messagetype);
    return OK;
  }
}

/* Signal handler for status / stop requests */
#ifndef ESP_PTP
static void ptp_signal_handler(int signo, FAR siginfo_t *siginfo,
                               FAR void *context) {
  FAR struct ptp_state_s *state = (FAR struct ptp_state_s *)siginfo->si_user;

  if (signo == SIGHUP) {
    state->stop = true;
  } else if (signo == SIGUSR1 && siginfo->si_value.sival_ptr) {
    state->status_req =
        *(FAR struct ptpd_statusreq_s *)siginfo->si_value.sival_ptr;
  }
}

static void ptp_setup_sighandlers(FAR struct ptp_state_s *state) {
  struct sigaction act;

  act.sa_sigaction = ptp_signal_handler;
  sigfillset(&act.sa_mask);
  act.sa_flags = SA_SIGINFO;
  act.sa_user = state;

  sigaction(SIGHUP, &act, NULL);
  sigaction(SIGUSR1, &act, NULL);
}
#endif // !ESP_PTP

/* Process status information request */

static void ptp_process_statusreq(FAR struct ptp_state_s *state) {
  FAR struct ptpd_status_s *status;

  if (!state->status_req.dest) {
    return; /* No active request */
  }

  status = state->status_req.dest;
  status->ptp_profile = state->ptp_profile;
  status->peer_is_endpoint = state->port[0].peer_is_endpoint;
  status->clock_source_valid = state->selected_source_valid;

  /* Copy own identity info to status struct */

  FAR struct ptp_announce_s *o = &state->own_identity;

  memcpy(status->own_identity_info.id, o->header.sourceidentity,
         sizeof(status->own_identity_info.id));

  status->own_identity_info.utcoffset = o->utcoffset[0];
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

  if (status->clock_source_valid) {
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
  status->last_delta_ns = state->last_delta_ns;
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
#ifdef ESP_PTP
static void ptp_daemon(void *task_param)
#else
static int ptp_daemon(int argc, FAR char **argv)
#endif // ESP_PTP
{
  FAR struct ptp_state_s *state;
#ifdef ESP_PTP
  struct pollfd pollfds[1]; // everything is received over one socket at L2
  struct ptp_bootstrap_args_s *args = (struct ptp_bootstrap_args_s *)task_param;
#else
  FAR const char *interface = "eth0";
  struct pollfd pollfds[2];
  struct msghdr rxhdr;
  struct iovec rxiov;
#endif // ESP_PTP
  int ret;

#ifndef ESP_PTP
  memset(&rxhdr, 0, sizeof(rxhdr));
  memset(&rxiov, 0, sizeof(rxiov));
#endif // !ESP_PTP

  state = calloc(1, sizeof(struct ptp_state_s));

#ifndef ESP_PTP
  if (argc > 1) {
    interface = argv[1];
  }
#endif // !ESP_PTP

#ifdef ESP_PTP
  int init_rc = ptp_initialize_state(state, args);
  free(args); /* heap struct from ptpd_start / ptpd_start_port */
  if (init_rc != OK) {
#else
  if (ptp_initialize_state(state, interface) != OK) {
#endif // ESP_PTP
    ptperr("Failed to initialize PTP state, exiting\n");

    ptp_destroy_state(state);
    free(state);

#ifdef ESP_PTP
    goto err;
#else
    return ERROR;
#endif // ESP_PTP
  }
#ifndef ESP_PTP
  ptp_setup_sighandlers(state);
#endif // !ESP_PTP

  pollfds[0].events = POLLIN;
#ifdef ESP_PTP
  pollfds[0].fd = state->port[0].ptp_socket;
#else
  pollfds[0].fd = state->event_socket;
  pollfds[1].events = POLLIN;
  pollfds[1].fd = state->info_socket;
#endif // ESP_PTP

  while (!state->stop) {
    ptpd_lateness_tick();
    state->port[0].can_send_delayreq = ptp_is_gptp(state);

#ifndef ESP_PTP
    rxhdr.msg_name = NULL;
    rxhdr.msg_namelen = 0;
    rxhdr.msg_iov = &rxiov;
    rxhdr.msg_iovlen = 1;
    rxhdr.msg_control = &state->rxcmsg;
    rxhdr.msg_controllen = sizeof(state->rxcmsg);
    rxhdr.msg_flags = 0;
    rxiov.iov_base = &state->port[0].rxbuf;
    rxiov.iov_len = sizeof(state->port[0].rxbuf);
#endif // !ESP_PTP

    pollfds[0].revents = 0;
#ifndef ESP_PTP
    pollfds[1].revents = 0;
    ret = poll(pollfds, 2, PTPD_POLL_INTERVAL);
#else
    ret = poll(pollfds, 1, PTPD_POLL_INTERVAL);
#endif // !ESP_PTP

    if (pollfds[0].revents) {
      /* Receive time-critical packet, potentially with cmsg
       * indicating the timestamp.
       */

#ifdef ESP_PTP
      ret = ptp_net_recv(state, &state->port[0].rxbuf,
                         sizeof(state->port[0].rxbuf), &state->port[0].rxtime);
#else
      ret = recvmsg(state->event_socket, &rxhdr, MSG_DONTWAIT);
#endif // ESP_PTP

      if (ret > 0) {
#ifndef ESP_PTP
        ptp_getrxtime(state, &rxhdr, &state->port[0].rxtime);
#endif
        ptp_process_rx_packet(state, ret);
      }
    }

#ifndef ESP_PTP
    if (pollfds[1].revents) {
      /* Receive non-time-critical packet. */

      ret = recv(state->info_socket, &state->port[0].rxbuf,
                 sizeof(state->port[0].rxbuf), MSG_DONTWAIT);
      if (ret > 0) {
        ptp_process_rx_packet(state, ret);
      }
    }

    if (pollfds[0].revents == 0 && pollfds[1].revents == 0) {
      /* No packets received, check for multicast timeout */

      ptp_check_multicast_status(state);
    }
#endif // !ESP_PTP
    ptp_periodic_send(state);
    ptp_check_profile_fallback(state);

    state->selected_source_valid = is_selected_source_valid(state);
    ptp_process_statusreq(state);
  } // while (!state->stop)
  ptp_destroy_state(state);
  free(state);

#ifdef ESP_PTP
err:
  s_state = NULL;
  vTaskDelete(NULL);
#else
  return 0;
#endif // ESP_PTP
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
#ifdef ESP_PTP
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
    if (xTaskCreate(ptp_daemon, "PTPD", CONFIG_NETUTILS_PTPD_STACKSIZE,
                    args, 6, NULL) != pdPASS) {
      free(args);
      ESP_LOGE(TAG, "ptpd_start_port: xTaskCreate failed");
      return -1;
    }
    return 1;
  }

  /* Attach to running daemon. */
  struct ptp_port_s *p = &s_state->port[port_index];
  p->enabled = true;
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
#else
  UNUSED(port_index);
  UNUSED(medium);
  return ptpd_start(interface);
#endif
}

int ptpd_inject_peer_delay(int port_index, int64_t peer_delay_ns) {
#ifdef ESP_PTP
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  /* Sanity-bound — single Wi-Fi RTT/2 is typically tens of µs; values
   * outside the standard wired bound suggest a measurement anomaly
   * (multipath, association handoff). Discard. */
  if (peer_delay_ns < 0 ||
      peer_delay_ns >= CONFIG_NETUTILS_PTPD_MAX_PEER_DELAY_NS) {
    return -ERANGE;
  }
  struct ptp_port_s *p = &s_state->port[port_index];

  /* Running average — same shape as the wired Pdelay handler at
   * ptp_process_delay_resp_followup (~line 2728). Caps the average
   * window at CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT samples, which
   * lets ptp_update_local_clock read a smoothed value via
   * port->peer_delay_ns without each FTM burst's jitter showing up
   * as a clock-jump. */
  if (p->peer_delay_avgcount < CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT) {
    p->peer_delay_avgcount++;
  }
  p->peer_delay_ns += (long)(peer_delay_ns - p->peer_delay_ns) /
                      p->peer_delay_avgcount;
  return OK;
#else
  UNUSED(port_index);
  UNUSED(peer_delay_ns);
  return -ENOSYS;
#endif
}

int ptpd_inject_sync(int port_index, FAR const uint8_t *follow_up_info,
                     size_t len) {
#ifdef ESP_PTP
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  if (follow_up_info == NULL || len < sizeof(struct ptp_follow_up_s)) {
    return -EINVAL;
  }

  /* The §12.7 IE payload IS an entire 802.1AS Follow_Up message —
   * header + preciseOriginTimestamp + FollowUpInformation TLV. We
   * receive it via the on-air carrier (Beacon Vendor IE, per the
   * carrier deviation documented in ESP-AVB-Bridge/project.md) at
   * some point AFTER the actual master TX. The receive instant is
   * "now" — t2 in §12.1.2 terms — and the FollowUpInformation
   * carries t1 (preciseOriginTimestamp). Peer-delay correction is
   * applied inside ptp_update_local_clock from port->peer_delay_ns,
   * which FTM injection will populate once the STA-side FTM client
   * is wired up. */
  const struct ptp_follow_up_s *fu =
      (const struct ptp_follow_up_s *)follow_up_info;

  /* Validate messagetype (low nibble = Follow_Up = 0x08; high nibble
   * carries gPTP majorSdoId on gPTP messages). */
  if ((fu->header.messagetype & PTP_MSGTYPE_MASK) != PTP_MSGTYPE_FOLLOW_UP) {
    return -EINVAL;
  }

  struct ptp_port_s *p = &s_state->port[port_index];

  /* Synthesise a selected_source on first injection so the rest of
   * the daemon (status APIs, BTCA "are we slave" tests) sees a valid
   * upstream. The Follow_Up doesn't carry the full Announce content,
   * but on Wi-Fi §12.1.2 there is no separate Announce stream — the
   * §12.7 IE flow is the only timing info. Mirror the Follow_Up's
   * sourceidentity into selected_source so identity-tracking works;
   * leave priority/quality at defaults (we accept whatever sends
   * us beacon-IE timing). */
  if (!s_state->selected_source_valid ||
      memcmp(s_state->selected_source.header.sourceidentity,
             fu->header.sourceidentity, 8) != 0) {
    memset(&s_state->selected_source, 0, sizeof(s_state->selected_source));
    s_state->selected_source.header = fu->header;
    memcpy(s_state->selected_source.btc_identity,
           fu->header.sourceidentity,
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
#else
  UNUSED(port_index);
  UNUSED(follow_up_info);
  UNUSED(len);
  return -ENOSYS;
#endif
}

int ptpd_inject_sync_pair(int port_index, int64_t remote_ns,
                          int64_t local_ns) {
#ifdef ESP_PTP
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  struct ptp_port_s *p = &s_state->port[port_index];

  /* Synthesise a selected_source if none yet — without it the daemon's
   * status APIs and BTCA "are we slave" tests reject the injection.
   * The caller already validated GM identity at the IE-parse layer
   * (the §12.7 IE sourcePortIdentity is checked separately via
   * ptpd_inject_sync's selected_source path); for pair-style FTM
   * sync where the IE bytes have been consumed elsewhere, we leave
   * the identity at whatever ptpd_inject_sync set. */
  if (!s_state->selected_source_valid) {
    return -ENOENT;
  }

  struct timespec remote_ts = {.tv_sec = (time_t)(remote_ns / NSEC_PER_SEC),
                               .tv_nsec = (long)(remote_ns % NSEC_PER_SEC)};
  struct timespec local_ts = {.tv_sec = (time_t)(local_ns / NSEC_PER_SEC),
                              .tv_nsec = (long)(local_ns % NSEC_PER_SEC)};

  clock_gettime(CLOCK_MONOTONIC, &p->last_received_sync);

  return ptp_update_local_clock(s_state, &remote_ts, &local_ts);
#else
  UNUSED(port_index);
  UNUSED(remote_ns);
  UNUSED(local_ns);
  return -ENOSYS;
#endif
}

int ptpd_register_sync_egress_cb(int port_index, ptpd_sync_egress_cb_t cb,
                                 FAR void *ctx) {
#ifdef ESP_PTP
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  if (s_state == NULL) {
    return -ESRCH;
  }
  s_state->port[port_index].sync_egress_cb = cb;
  s_state->port[port_index].sync_egress_ctx = ctx;
  return OK;
#else
  UNUSED(port_index);
  UNUSED(cb);
  UNUSED(ctx);
  return -ENOTSUP;
#endif
}

/* Software-clock indirection. When ptp_clock_sw_init() registers a
 * software clock backend (used on chips without IEEE 1588 hardware,
 * e.g. ESP32-C6), s_sw_clock_now is set to the backend's reader and
 * ptpd_now() routes through it instead of clock_gettime().
 *
 * On P4 wired builds nothing initializes the software clock and
 * ptpd_now() is a thin wrapper around clock_gettime(CLOCK_PTP_SYSTEM)
 * with no added cost. */
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
#ifdef ESP_PTP
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
  if (xTaskCreate(ptp_daemon, "PTPD", CONFIG_NETUTILS_PTPD_STACKSIZE,
                  args, 6, NULL) != pdPASS) {
    free(args);
    ESP_LOGE(TAG, "ptpd_start: xTaskCreate failed");
    return -1;
  }
  return 1;
#else
  int pid;
  FAR char *task_argv[] = {(FAR char *)interface, NULL};

  pid = task_create("PTPD", CONFIG_NETUTILS_PTPD_SERVERPRIO,
                    CONFIG_NETUTILS_PTPD_STACKSIZE, ptp_daemon, task_argv);

  /* Use kill with signal 0 to check if the process is still alive
   * after initialization.
   */

  usleep(USEC_PER_TICK);
  if (kill(pid, 0) != OK) {
    return ERROR;
  } else {
    return pid;
  }
#endif // ESP_PTP
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
 *   Multiple threads with priority less than CONFIG_NETUTILS_PTPD_SERVERPRIO
 *   can request status simultaneously. If higher priority threads request
 *   status simultaneously, some of the requests may timeout.
 *
 ****************************************************************************/

int ptpd_set_profile(int pid, ptp_profile_e profile) {
#ifdef ESP_PTP
  UNUSED(pid);

  if (profile != ptp_profile_standard && profile != ptp_profile_gptp) {
    return -EINVAL;
  }

  if (s_state == NULL) {
    return -ESRCH;
  }

  if (s_state->ptp_profile != profile) {
    s_state->ptp_profile = profile;
    ptp_reset_for_profile(s_state);
    if (profile == ptp_profile_gptp) {
      ptp_arm_profile_fallback(s_state);
    }
    ptpinfo("PTP profile changed to %s mode.\n",
            profile == ptp_profile_gptp ? "gPTP" : "standard");
  }

  return OK;
#else
  UNUSED(pid);
  UNUSED(profile);
  return -ENOTSUP;
#endif
}

int ptpd_status(int pid, FAR struct ptpd_status_s *status) {
#ifdef ESP_PTP
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
#endif
#ifndef CONFIG_BUILD_FLAT

  /* TODO: Use SHM memory to pass the status information if processes
   * do not share the same memory space.
   */

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
#ifdef ESP_PTP
  s_state->stop = true;
  return OK;
#else
  if (kill(pid, SIGHUP) == OK) {
    return OK;
  } else {
    return -errno;
  }
#endif
}
