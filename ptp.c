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

/* True once ptp_clock_sw_init() succeeds; routes time ops through the
 * SW clock backend. On C6 this is the only path that can actually move
 * the local clock rate (no working clock_adjtime(ADJ_FREQUENCY)). */
static bool s_use_sw_clock = false;

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
#ifndef CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS
#define CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS 16000
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
#define PTP_FREQ_P_DIV 100
#define PTP_FREQ_I_DIV 1000
typedef struct {
  int32_t kp;
  int32_t ki;
  int32_t drift_acc;
} pi_cntrl_t;

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

  ptp_profile_e ptp_profile;

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

  /* gPTP → standard PTP fallback gate. One-shot per session: set true after
   * either AVB Lite §2.2 condition fires; cleared on Ethernet link-up so the
   * check re-arms for the new link. */

  bool gptp_fallback_done;
  bool eth_event_handler_registered;
  bool wifi_event_handler_registered;
};

#ifdef CONFIG_ESP_PTP_SERVER
#define PTPD_POLL_INTERVAL CONFIG_ESP_PTP_SYNC_INTERVAL_MS
#else
#define PTPD_POLL_INTERVAL CONFIG_ESP_PTP_TIMEOUT_MS
#endif

/* PTP debug messages are enabled by either CONFIG_DEBUG_NET_INFO
 * or separately by CONFIG_ESP_PTP_DEBUG. This simplifies
 * debugging without having excessive amount of logging from net.
 */

static const char *TAG = "ptpd";
#define ptpinfo(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define ptpwarn(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define ptperr(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

static struct ptp_state_s *s_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
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
      ptp_set_port_link(state, port, true);
    }
    break;
  case WIFI_EVENT_STA_DISCONNECTED:
    if (p->wifi_mode == ptp_port_wifi_mode_sta) {
      ptp_set_port_link(state, port, false);
    }
    break;
  case WIFI_EVENT_AP_START:
    if (p->wifi_mode == ptp_port_wifi_mode_ap) {
      ptp_set_port_link(state, port, true);
    }
    break;
  case WIFI_EVENT_AP_STOP:
    if (p->wifi_mode == ptp_port_wifi_mode_ap) {
      ptp_set_port_link(state, port, false);
    }
    break;
  default:
    break;
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

  if (ret <= ETH_HEADER_LEN) {
    return ERROR;
  }

  size_t payload_len = (size_t)ret - ETH_HEADER_LEN;
  if (payload_len > ptp_msg_len) {
    payload_len = ptp_msg_len;
  }

  memcpy(ptp_msg, &eth_frame[ETH_HEADER_LEN], payload_len);

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
  int system_identity_check = 1;

  if (a->btc_priority1 < b->btc_priority1      /* Main priority field */
      || a->btc_quality[0] < b->btc_quality[0] /* Clock class */
      || a->btc_quality[1] < b->btc_quality[1] /* Clock accuracy */
      || a->btc_quality[2] < b->btc_quality[2] /* Clock variance high byte */
      || a->btc_quality[3] < b->btc_quality[3] /* Clock variance low byte */
      || a->btc_priority2 < b->btc_priority2   /* Sub priority field */
      ||
      memcmp(a->btc_identity, b->btc_identity, sizeof(a->btc_identity)) < 0) {
    system_identity_check = -1;
  } else if (a->btc_priority1 == b->btc_priority1      /* Main priority field */
             && a->btc_quality[0] == b->btc_quality[0] /* Clock class */
             && a->btc_quality[1] == b->btc_quality[1] /* Clock accuracy */
             && a->btc_quality[2] ==
                    b->btc_quality[2] /* Clock variance high byte */
             && a->btc_quality[3] ==
                    b->btc_quality[3] /* Clock variance low byte */
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
   *
   * Use the more recent of last_received_sync and last_received_announce.
   * Wired endpoints get Sync at 8 Hz and refresh last_received_sync
   * continuously; Wi-Fi STAs receiving §12.2 Announces (no Sync on the
   * 0x88f7 socket) refresh last_received_announce continuously, while
   * last_received_sync is only updated on BTCA *switch* events. Looking
   * at both prevents the Wi-Fi case from spuriously timing out after
   * CONFIG_ESP_PTP_TIMEOUT_MS while a stable source is still
   * being announced. */
  clock_gettime(CLOCK_MONOTONIC, &time_now);
  struct timespec last_evt = state->port[0].last_received_sync;
  if (state->port[0].last_received_announce.tv_sec > last_evt.tv_sec ||
      (state->port[0].last_received_announce.tv_sec == last_evt.tv_sec &&
       state->port[0].last_received_announce.tv_nsec > last_evt.tv_nsec)) {
    last_evt = state->port[0].last_received_announce;
  }
  clock_timespec_subtract(&time_now, &last_evt, &delta);

  if (timespec_to_ms(&delta) > CONFIG_ESP_PTP_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Too long time since received packet\n");
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

  /* STA-mode wifi_ftm port: hand off the §12.7 beacon-IE parser, the
   * FTM initiator loop, and the FTM_REPORT handler to the dedicated
   * Wi-Fi STA transport module. Application code (e.g.
   * ESP-AVB-Endpoint) no longer needs to know about §12.7 IE bytes or
   * FTM cadence — bringing the port up is the only integration call. */
  if (p->wifi_mode == ptp_port_wifi_mode_sta) {
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
  state->ptp_profile = ptp_profile_gptp;
#else
  state->ptp_profile = ptp_profile_standard;
#endif
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
  struct timespec ts;
  int ret;

  memset(&msg, 0, sizeof(msg));
  msg = state->own_identity;
  msg.header.messagetype = PTP_MSGTYPE_ANNOUNCE;
  msg.header.messagelength[1] = sizeof(msg);
  msg.header.logmessageinterval =
      msec_to_log_period(CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS);

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

  ret = ptp_net_send(state, &msg, sizeof(msg), NULL);

  if (ret < 0) {
    ptperr("sendto failed: %d", errno);
  } else {
    ptpinfo("Sent announce, seq %ld\n", (long)ptp_get_sequence(&msg.header));
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
                                    FAR struct ptp_follow_up_s *out) {
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

  /* sourceClockIdentity: upstream BTC if synced, else our own. */
  if (state->selected_source_valid) {
    memcpy(out->header.sourceidentity, state->selected_source.btc_identity,
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
  tlv->type[1] = 3;      /* Organization extension */
  tlv->length[1] = 0x1c; /* 28 bytes of value */
  tlv->orgidentity[0] = 0x00;
  tlv->orgidentity[1] = 0x80;
  tlv->orgidentity[2] = 0xc2; /* IEEE 802.1 OUI */
  tlv->orgsubtype[2] = 1;     /* FollowUp Information per §11.4.4.3 */
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

  ptpinfo("Sent sync + follow-up, seq %ld\n",
          (long)ptp_get_sequence(&msg.header));
#else
  ptpinfo("Sent sync, seq %ld\n", (long)ptp_get_sequence(&msg.header));
#endif /* CONFIG_ESP_PTP_TWOSTEP_SYNC */

  return OK;
}

/* Send delay request packet to selected source */

static int ptp_send_delay_req(FAR struct ptp_state_s *state) {
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

  if (ptp_is_gptp(state)) {
    req.header.messagetype = PTP_MSGTYPE_PDELAY_REQ | PTP_MSGTYPE_SDOID_GPTP;
    req.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
    req.header.controlfield = 5;
    req_len = sizeof(struct ptp_pdelay_req_s);

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

  ret = ptp_net_send(state, &req, req_len, &state->port[0].delayreq_time);

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
#if defined(CONFIG_ESP_PTP_SERVER) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE)
  /* If there is no better timetransmitter clock on the network,
   * act as the reference source and send server packets. */

  /* "I am BTC" Announce + Sync via port[0]'s L2TAP socket; only valid
   * on an eth_hwts port. wifi_ftm uses the egress-cb path below. */
  if (!state->selected_source_valid && state->port[0].enabled &&
      state->port[0].medium == ptp_port_medium_eth_hwts &&
      state->port[0].ptp_socket >= 0) {
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
    if (timespec_to_ms(&delta) > CONFIG_ESP_PTP_SYNC_INTERVAL_MS) {
      state->port[0].last_transmitted_sync = time_now;
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
    ptp_marshal_follow_up_for_beacon_ie(state, &fu);
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
    if (timespec_to_ms(&delta) <= CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS) {
      continue;
    }
    port->last_transmitted_announce = time_now;

    /* Build Announce body. Prefer the selected upstream source's
     * identity/quality if we have one (bridge relay case); otherwise
     * publish our own (we are the BTC). Matches ptp_send_announce's
     * layout (header + body + path-trace TLV). */
    struct ptp_announce_s msg;
    memset(&msg, 0, sizeof(msg));
    if (state->selected_source_valid) {
      msg = state->selected_source;
    } else {
      msg = state->own_identity;
    }
    msg.header.messagetype = PTP_MSGTYPE_ANNOUNCE;
    msg.header.messagelength[1] = sizeof(msg);
    msg.header.logmessageinterval =
        msec_to_log_period(CONFIG_ESP_PTP_ANNOUNCE_INTERVAL_MS);
    if (ptp_is_gptp(state)) {
      msg.header.messagetype |= PTP_MSGTYPE_SDOID_GPTP;
      msg.header.flags[1] = PTP_FLAGS1_PTP_TIMESCALE;
    }
    ptp_increment_sequence(&state->announce_seq, &msg.header);
    struct timespec origin_ts;
    ptp_gettime(state, &origin_ts);
    timespec_to_ptp_format(&origin_ts, msg.origintimestamp);
    struct ptp_pathtrace_tlv_s pathtrace_tlv;
    memset(&pathtrace_tlv, 0, sizeof(pathtrace_tlv));
    pathtrace_tlv.type[1] = 8;
    pathtrace_tlv.length[1] = 8;
    memcpy(pathtrace_tlv.pathsequence,
           state->selected_source_valid ? state->selected_source.btc_identity
                                        : state->own_identity.btc_identity,
           sizeof(pathtrace_tlv.pathsequence));
    msg.pathtracetlv = pathtrace_tlv;

    ptp_wifi_ap_send_announce(p, port->intf_hw_addr, &msg, sizeof(msg));
  }

  /* Post-fallback endpoint beacon (profiles/avb_lite.md §2.3): after
   * fallback to standard PTP, periodically emit Pdelay_Req-with-TLV to
   * the gPTP bridge-group MAC so gPTP peers can detect us and follow.
   * 3 s cadence matches the §2.2 evaluation window. */

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

  /* The ESP-IDF hardware clock applies ADJ_FREQUENCY relative to its current
   * addend. Keep the PI output as an absolute trim and apply only its delta;
   * applying the complete output every Sync accumulates rate error. */
  state->offset_pi.drift_acc += offset_ns / PTP_FREQ_I_DIV;
  if (state->offset_pi.drift_acc > ADJ_FREQ_MAX) {
    state->offset_pi.drift_acc = ADJ_FREQ_MAX;
  } else if (state->offset_pi.drift_acc < -ADJ_FREQ_MAX) {
    state->offset_pi.drift_acc = -ADJ_FREQ_MAX;
  }
  int64_t target_trim = offset_ns / PTP_FREQ_P_DIV + state->offset_pi.drift_acc;
  if (target_trim > ADJ_FREQ_MAX) {
    target_trim = ADJ_FREQ_MAX;
  } else if (target_trim < -ADJ_FREQ_MAX) {
    target_trim = -ADJ_FREQ_MAX;
  }
  int32_t trim_delta = (int32_t)target_trim - state->freq_trim_ppb;

  /* Record interval diagnostics; these values are not an adjustment input
   * because short Sync-to-Sync measurements contain scheduling jitter. */
  int64_t remote_time_ns = timespec_to_ns(remote_timestamp);
  int64_t local_time_ns = timespec_to_ns(local_timestamp);
  int64_t remote_delta_ns = remote_time_ns - state->remote_time_ns_prev;
  int64_t local_delta_ns = local_time_ns - state->local_time_ns_prev;
  int64_t tick_diff = remote_delta_ns - local_delta_ns;

  if (s_use_sw_clock) {
    /* SW backend caps at +/- 1e8 ppb; let it clamp by returning -1. */
    ptp_clock_sw_adjtime_rate((int32_t)trim_delta);
    if (ptp_clock_sw_adjtime_rate((int32_t)trim_delta) == 0) {
      state->freq_trim_ppb = (int32_t)target_trim;
    }
  } else {
    struct timex tx = {
        .modes = ADJ_FREQUENCY,
        .freq = trim_delta,
    };
    clock_adjtime(PTPD_CLOCK_ID, &tx);
    if (clock_adjtime(PTPD_CLOCK_ID, &tx) == 0) {
      state->freq_trim_ppb = (int32_t)target_trim;
    }
  }

  state->remote_time_ns_prev = remote_time_ns;
  state->local_time_ns_prev = local_time_ns;

  ptpinfo("remote_delta_ns %lli, local_delta_ns %lli, tick_diff %lli\n",
          remote_delta_ns, local_delta_ns, tick_diff);
  ptpinfo("offset_ns %lli, trim %li, delta %li, drift_acc %li\n", offset_ns,
          (long)target_trim, (long)trim_delta,
          (long)state->offset_pi.drift_acc);
  // Get the path delay only when clock is stable enough. If we were
  // in the process of speeding/slowing the local clock, we'd get an
  // incorrect delay measurement
  int64_t diff = llabs(offset_ns) - llabs(state->last_offset_ns);
  static int cnt = 0;
  if ((ptp_is_gptp(state) &&
       llabs(diff) < CONFIG_ESP_PTP_PEER_DELAY_STABILITY_NS) ||
      (!ptp_is_gptp(state) &&
       llabs(diff) < CONFIG_ESP_PTP_PATH_DELAY_STABILITY_NS)) {
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

  ptpinfo("Local time: %lld.%09ld, remote time %lld.%09ld\n",
          (long long)local_timestamp->tv_sec, (long)local_timestamp->tv_nsec,
          (long long)remote_timestamp->tv_sec, (long)remote_timestamp->tv_nsec);

  delta_ns = timespec_delta_ns(remote_timestamp, local_timestamp);
  if (ptp_is_gptp(state)) {
    delta_ns += state->port[0].peer_delay_ns + state->correction_ns;
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
             sizeof(msg->header.sourceidentity)) != 0) {
    /* This packet wasn't from the currently selected source */
    ESP_LOGD(TAG, "This packet wasn't from the currently selected source");
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
  if (ptp_is_gptp(state)) {
    state->correction_ns = get_correction_ns(msg->header.correction);
  }
  return ptp_update_local_clock(state, &remote_time,
                                &state->port[0].twostep_rxtime);
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

#if defined(CONFIG_ESP_PTP_TWOSTEP_SYNC) ||                              \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE)
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
  /* gPTP requires the 0x7F "no change" sentinel here per IEEE
   * 1588-2008 §13.6.2.2 and 802.1AS-2011 §11.4.5.4 / §11.4.5.5;
   * neither message dictates the requester's Pdelay_Req cadence.
   * Pdelay_Resp_Follow_Up inherits the value from this header. */
  resp.header.logmessageinterval = msec_to_log_period(
      ptp_is_gptp(state) ? log_period_to_msec(0x7F)
                         : CONFIG_ESP_PTP_DELAYREQ_INTERVAL_MS);
  resp.header.messagelength[1] = resp_len;

  /* Send the response message */

  ret = ptp_net_send(state, &resp, resp_len, &ts);

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
    resp.header.messagelength[1] = fup_len;
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

  /* gPTP peer_delay = ((t4 - t1) - (t3 - t2)) / 2 */

  peer_delay_roundtrip = timespec_delta_ns(
      &state->port[0].twostep_delay_resp_rxtime, &state->port[0].delayreq_time);
  ptp_format_to_timespec(
      state->port[0].twostep_delay_resp_packet.receivetimestamp,
      &remote_rxtime);
  ptp_format_to_timespec(msg->origintimestamp, &remote_txtime);
  peer_delay_reflection = timespec_delta_ns(&remote_txtime, &remote_rxtime);
  peer_delay = (peer_delay_roundtrip - peer_delay_reflection) / 2;

  if (peer_delay >= 0 && peer_delay < CONFIG_ESP_PTP_MAX_PEER_DELAY_NS) {
    if (state->port[0].peer_delay_avgcount <
        CONFIG_ESP_PTP_DELAYREQ_AVGCOUNT) {
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

  if (state->port[0].rxbuf.header.domain != CONFIG_ESP_PTP_DOMAIN) {
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
#if defined(CONFIG_ESP_PTP_CLIENT) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always acts as a client
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

#if defined(CONFIG_ESP_PTP_SERVER) ||                                    \
    defined(CONFIG_ESP_PTP_GPTP_PROFILE) // gPTP always responds to delay
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
  int poll_timeout =
      (pollfds[0].fd < 0) ? PTPD_NOSOCK_POLL_MS : PTPD_POLL_INTERVAL;

  while (!state->stop) {
    ptpd_lateness_tick();
    state->port[0].can_send_delayreq = ptp_is_gptp(state);

    pollfds[0].revents = 0;
    ret = poll(pollfds, 1, poll_timeout);

    if (pollfds[0].revents) {
      /* Receive time-critical packet, potentially with cmsg
       * indicating the timestamp.
       */

      ret = ptp_net_recv(state, &state->port[0].rxbuf,
                         sizeof(state->port[0].rxbuf), &state->port[0].rxtime);

      if (ret > 0) {
        ptp_process_rx_packet(state, ret);
      }
    }

    ptp_periodic_send(state);
    ptp_check_profile_fallback(state);

    state->selected_source_valid = is_selected_source_valid(state);
    ptp_process_statusreq(state);
  } // while (!state->stop)
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

/* Push a Wi-Fi-received PTP frame (Ethernet header already stripped)
 * into the daemon's RX path. Used by per-medium modules that don't
 * have an L2TAP socket. The frame is copied into
 * port[0]'s rxbuf and routed through the same ptp_process_rx_packet
 * the EMAC RX loop uses, so received Announce / Sync / Follow_Up
 * messages flow into the existing BMCA / servo state machine. */
int ptp_inject_received_frame(int port_index, const uint8_t *frame,
                              uint16_t len) {
  if (s_state == NULL) {
    return -ESRCH;
  }
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS) {
    return -EINVAL;
  }
  if (len < sizeof(struct ptp_header_s) ||
      len > sizeof(s_state->port[port_index].rxbuf.raw)) {
    return -EINVAL;
  }
  /* Quick gate: only forward PTP frames whose profile matches ours.
   * Avoids waking BMCA on stray non-gPTP traffic. Returns +1 to
   * distinguish "filtered by profile mismatch" from "processed OK". */
  uint8_t mt = frame[0];
  bool msg_is_gptp = (mt & PTP_MSGTYPE_SDOID_GPTP) != 0;
  if (msg_is_gptp != ptp_is_gptp(s_state)) {
    return 1;
  }
  memcpy(s_state->port[0].rxbuf.raw, frame, len);
  /* Wi-Fi RX has no MAC-layer hardware timestamp surfaced to user
   * space — best we can do is "now" in the local PTP-disciplined
   * clock. Announce processing doesn't use rxtime, so this is only
   * relevant if Sync / Follow_Up arrive on this path (rare on a
   * §12.1.2 Wi-Fi link where Sync rides the beacon IE instead). */
  ptp_gettime(s_state, &s_state->port[0].rxtime);
  return ptp_process_rx_packet(s_state, len);
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
