/*
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Wi-Fi PTP transport for ports with medium = wifi_ftm. Combines the
 * AP-side (wifi_mode = ap) and STA-side (wifi_mode = sta) roles:
 *
 *   AP side (bridge):
 *     - ptp_wifi_ap_send_announce: re-emits the upstream BTC's
 *       Announce as one unicast 802.1AS frame per associated STA per
 *       IEEE 802.1AS-2020 §12.2 (the §12.7 beacon IE only carries
 *       Sync timing, not GM priority / clockQuality).
 *
 *   STA side (endpoint):
 *     - beacon Vendor IE callback that decodes the §12.7
 *       FollowUpInformation and the Scramble Tools-private (gPTP, TSF)
 *       mapping IE, feeding the daemon via the internal inject path;
 *     - FTM initiator task that bursts the AP at the §12.8.2 cadence
 *       and computes (BTC time, local RX time) pairs from
 *       (gptp_marker, tsf_marker) + per-burst HW t1/t2 timestamps;
 *     - WIFI_EVENT_FTM_REPORT handler that consumes the report.
 *
 * §12.2 unicast Announce/Sync/Follow_Up RX is NOT owned here — those
 * frames arrive on the IDF Wi-Fi rxcb (which the application's
 * dispatcher owns, since the rxcb is single-slot per interface) and
 * are fed into the daemon via the public ptp_inject_received_frame()
 * API in esp_ptp.h.
 *
 * Application code brings the port up via ptpd_start_port(.., wifi_ftm)
 * and never sees §12.7 IE bytes, FTM cadence, or raw PTP frames.
 */

#include "sdkconfig.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_ptp.h"
#include "ptp.h"
#include "ptp_wifi_association.h"
#include "ptp_path_trace.h"
#include "ptp_timing_snapshot.h"
#include "ptp_signaling.h"
#include "ptp_ftm_counter.h"
#include "ptp_ftm_session.h"
#include "ptp_ftm_rtt.h"
#include "ptp_ftm_success.h"
#include "ptp_ftm_clock.h"
#include "ptp_rpc_proto.h"

/* esp_wifi_internal_tx is not in any public IDF header. Same forward-
 * declaration pattern as in esp_avb/avbnet.c. The wifi_remote layer
 * on a host with a coprocessor radio forwards this call over SDIO so
 * the AP actually emits the frame from the C6 side. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

#define TAG "ptp_wifi"

#define ETH_HDR_LEN 14

/* ===========================================================================
 * AP side — §12.2 unicast Announce egress
 * ===========================================================================
 */

/* Build an Ethernet frame with PTP ethertype carrying ptp_msg, then
 * push it as one unicast TX to dst_mac via WIFI_IF_AP. Caller-owned
 * buffer; we allocate on stack since Announce is small (~88 B). */
static int wifi_ap_send_unicast_ptp(const uint8_t src_mac[6],
                                    const uint8_t dst_mac[6], void *ptp_msg,
                                    uint16_t ptp_msg_len) {
  uint8_t frame[ETH_HDR_LEN + ptp_msg_len];

  /* Ethernet header: dst MAC, src MAC (port's AP MAC), ethertype 0x88F7. */
  memcpy(frame + 0, dst_mac, 6);
  memcpy(frame + 6, src_mac, 6);
  frame[12] = 0x88;
  frame[13] = 0xF7;
  memcpy(frame + ETH_HDR_LEN, ptp_msg, ptp_msg_len);

  /* WIFI_IF_AP = 1 (numeric constant; the wifi_remote/native split
   * doesn't expose a stable header constant we can pull in without
   * tightening the build dependency further). */
  esp_err_t r = esp_wifi_internal_tx(1, frame, sizeof(frame));
  return (r == ESP_OK) ? (int)sizeof(frame) : -1;
}

#define PTP_AP_ANNOUNCE_MAX PTP_ANNOUNCE_MAX_LENGTH
typedef struct {
  int port_index;
  uint32_t generation;
  uint32_t source_generation;
  int64_t queued_us;
  ptp_wifi_association_snapshot_t peers;
  int8_t initial_interval;
  uint8_t src_mac[6];
  uint16_t len;
  uint8_t msg[PTP_AP_ANNOUNCE_MAX];
} ap_announce_item_t;

/* Owned by the one Announce worker. Ignore timestamps and sequence changes. */
static struct {
  uint32_t revision;
  uint16_t length;
  uint8_t message[PTP_AP_ANNOUNCE_MAX];
} s_announce_information[CONFIG_ESP_PTP_NUM_PORTS];

static uint32_t ap_announce_information(ap_announce_item_t *item) {
  if (item->port_index < 0 || item->port_index >= CONFIG_ESP_PTP_NUM_PORTS) return 0;
  unsigned port = item->port_index;
  const uint8_t *previous = s_announce_information[port].message;
  if (!s_announce_information[port].revision || s_announce_information[port].length != item->len ||
      previous[4] != item->msg[4] || memcmp(previous + 6, item->msg + 6, 2) ||
      memcmp(previous + 20, item->msg + 20, 8) ||
      memcmp(previous + 44, item->msg + 44, item->len - 44)) {
    memcpy(s_announce_information[port].message, item->msg, item->len);
    s_announce_information[port].length = item->len;
    if (!++s_announce_information[port].revision) ++s_announce_information[port].revision;
  }
  return s_announce_information[port].revision;
}

static int ap_send_announce_now(ap_announce_item_t *item) {
  int64_t started_us = esp_timer_get_time();
  if (started_us < item->queued_us || started_us - item->queued_us >= 1000000 ||
      item->generation != ptp_wifi_link_generation(item->port_index) ||
      item->source_generation != ptpd_timing_generation()) return 0;
  uint32_t information_revision = ap_announce_information(item);
  if (!information_revision) return 0;
  int sent = 0;
  for (unsigned index = 0; index < item->peers.count; ++index) {
    int64_t now_us = esp_timer_get_time();
    if (now_us < item->queued_us || now_us - item->queued_us >= 1000000 ||
        item->generation != ptp_wifi_link_generation(item->port_index) ||
        item->source_generation != ptpd_timing_generation()) break;
    const ptp_wifi_association_entry_t *peer = &item->peers.entries[index];
    int8_t advertised;
    uint16_t sequence;
    uint32_t token = ptpd_wifi_announce_begin(item->port_index, peer->mac,
        peer->association, peer->announce_revision, information_revision,
        item->initial_interval, now_us, &advertised, &sequence);
    if (!token) continue;
    item->msg[28] = peer->port_number >> 8;
    item->msg[29] = peer->port_number;
    item->msg[30] = sequence >> 8;
    item->msg[31] = sequence;
    item->msg[33] = (uint8_t)advertised;
    bool accepted = wifi_ap_send_unicast_ptp(item->src_mac, peer->mac, item->msg, item->len) > 0;
    ptpd_wifi_announce_finish(item->port_index, peer->mac, peer->association,
                              token, accepted, esp_timer_get_time());
    if (accepted) ++sent;
  }
  return sent;
}

static QueueHandle_t s_announce_mbox;

static void ap_announce_task(void *arg) {
  (void)arg;
  ap_announce_item_t item;
  for (;;) {
    if (xQueueReceive(s_announce_mbox, &item, portMAX_DELAY) == pdTRUE) {
      ap_send_announce_now(&item);
    }
  }
}

/* Copy peer identities with the message; the worker does no station-list RPC. */
int ptp_wifi_ap_send_announce(int port_index, const uint8_t src_mac[6],
                              void *ptp_msg, uint16_t ptp_msg_len) {
  if (!src_mac || !ptp_msg || ptp_msg_len < PTP_ANNOUNCE_BODY_LENGTH ||
      ptp_msg_len > PTP_AP_ANNOUNCE_MAX) return -1;
  ap_announce_item_t item = {.port_index = port_index,
      .generation = ptp_wifi_link_generation(port_index),
      .source_generation = ptpd_timing_generation(),
      .queued_us = esp_timer_get_time(), .len = ptp_msg_len};
  if (!ptpd_wifi_association_snapshot(port_index, &item.peers)) return -1;
  if (!item.peers.count) return 0;
  memcpy(item.src_mac, src_mac, sizeof(item.src_mac));
  memcpy(item.msg, ptp_msg, ptp_msg_len);
  item.initial_interval = (int8_t)item.msg[33];
  if (s_announce_mbox == NULL) {
    s_announce_mbox = xQueueCreate(1, sizeof(ap_announce_item_t));
    if (s_announce_mbox == NULL) return -1;
    if (xTaskCreatePinnedToCore(ap_announce_task, "ptp_ann_pub", 4096, NULL, 5,
                                NULL, 0) != pdPASS) {
      vQueueDelete(s_announce_mbox);
      s_announce_mbox = NULL;
      return -1;
    }
  }
  return xQueueOverwrite(s_announce_mbox, &item) == pdTRUE ? 0 : -1;
}

typedef struct {
  int port_index;
  bool ap;
  ptp_wifi_capable_result_t result;
  uint32_t generation;
  int64_t queued_us;
  uint8_t source[6];
  uint8_t message[PTP_CAPABLE_MESSAGE_LENGTH];
} wifi_capable_item_t;

static QueueHandle_t s_capable_queue;
static QueueHandle_t s_capable_results;
#define WIFI_CAPABLE_QUEUE_DEPTH (16 * CONFIG_ESP_PTP_NUM_PORTS)

static bool wifi_capable_current(const wifi_capable_item_t *item) {
  int64_t age = esp_timer_get_time() - item->queued_us;
  return age >= 0 && age < 1000000 &&
      item->generation == ptp_wifi_link_generation(item->port_index);
}

static bool wifi_capable_send_to(const wifi_capable_item_t *item,
                                 const uint8_t destination[6]) {
  if (!wifi_capable_current(item)) return false;
  uint8_t frame[ETH_HDR_LEN + PTP_CAPABLE_MESSAGE_LENGTH];
  memcpy(frame, destination, 6);
  memcpy(frame + 6, item->source, 6);
  frame[12] = 0x88;
  frame[13] = 0xf7;
  memcpy(frame + ETH_HDR_LEN, item->message, sizeof(item->message));
#ifdef CONFIG_ESP_PTP_CAPABLE_TX_TRACE
  int64_t submitted_us = esp_timer_get_time();
#endif
  int result = esp_wifi_internal_tx(item->ap ? 1 : 0, frame, sizeof(frame));
#ifdef CONFIG_ESP_PTP_CAPABLE_TX_TRACE
  ESP_LOGI("cap_probe", "CAPTX,%lld,%lld,%lld,%u,%d,%d", item->queued_us,
      submitted_us, esp_timer_get_time(),
      ((unsigned)item->message[30] << 8) | item->message[31],
      (int)(int8_t)item->message[54], result);
#endif
  return result == ESP_OK;
}

static void wifi_capable_task(void *argument) {
  (void)argument;
  wifi_capable_item_t item;
  for (;;) {
    if (xQueueReceive(s_capable_queue, &item, portMAX_DELAY) != pdTRUE) continue;
    item.result.accepted = wifi_capable_send_to(&item, item.result.destination);
    item.result.completed_us = esp_timer_get_time();
    xQueueSend(s_capable_results, &item.result, 0);
  }
}

/* Daemon is the sole producer. Queue saturation drops this interval's message. */
static bool wifi_capable_prepare(void) {
  if (s_capable_queue) return true;
  s_capable_queue = xQueueCreate(WIFI_CAPABLE_QUEUE_DEPTH, sizeof(wifi_capable_item_t));
  s_capable_results = xQueueCreate(WIFI_CAPABLE_QUEUE_DEPTH, sizeof(ptp_wifi_capable_result_t));
  if (s_capable_queue && s_capable_results &&
      xTaskCreatePinnedToCore(wifi_capable_task, "ptp_cap_pub", 4096, NULL,
                              5, NULL, 0) == pdPASS) return true;
  if (s_capable_queue) vQueueDelete(s_capable_queue);
  if (s_capable_results) vQueueDelete(s_capable_results);
  s_capable_queue = NULL;
  s_capable_results = NULL;
  return false;
}

int ptp_wifi_send_capable_peer(int port_index, bool ap, const uint8_t src_mac[6],
    const uint8_t destination[6], uint32_t generation, uint32_t association,
    uint32_t token, const uint8_t *message, uint16_t length) {
  if (port_index < 0 || port_index >= CONFIG_ESP_PTP_NUM_PORTS || !src_mac ||
      !destination || !association || !token || !message ||
      length != PTP_CAPABLE_MESSAGE_LENGTH || !wifi_capable_prepare()) return -1;
  wifi_capable_item_t item = {.port_index = port_index, .ap = ap,
      .generation = generation, .queued_us = esp_timer_get_time(),
      .result = {.port_index = port_index, .generation = generation,
                 .association = association, .token = token}};
  memcpy(item.result.destination, destination, 6);
  memcpy(item.source, src_mac, sizeof(item.source));
  memcpy(item.message, message, sizeof(item.message));
  return xQueueSend(s_capable_queue, &item, 0) == pdTRUE ? 0 : -1;
}

bool ptp_wifi_capable_result(ptp_wifi_capable_result_t *result) {
  return result && s_capable_results && xQueueReceive(s_capable_results, result, 0) == pdTRUE;
}

/* ===========================================================================
 * STA side — §12.7 beacon-IE consumer + §12.8.2 FTM initiator
 * ===========================================================================
 */

/* FTM cadence target. IEEE 802.1AS-2020 §12.8.2 sets
 * initialLogSyncInterval = -3 → 8 messages/s on Wi-Fi. ESP-IDF FTM
 * uses burst_period in 100 ms units (allowed: 0=No pref, 2..100). We
 * use 2 (= 200 ms ≈ 5 Hz) since 1 is below the documented minimum.
 * frm_count: 0(No pref), 16, 24, 32, 64. */
#define FTM_BURST_PERIOD_100MS 2
#define FTM_FRM_COUNT 16

/* Optional FTM transport consumes the same driver-owned report exactly once. */
extern bool ptp_ftm_report_hook(const wifi_event_ftm_report_t *report) __attribute__((weak));
extern void ptp_ftm_begin_hook(const uint8_t peer[6]) __attribute__((weak));
extern bool ptp_ftm_discipline_enabled(void) __attribute__((weak));
extern void ptp_ftm_reset_hook(void) __attribute__((weak));
extern bool ptp_ftm_status_hook(ptp_ftm_status_t *status) __attribute__((weak));

static int s_port_index = -1;
static EventGroupHandle_t s_events;
#define BIT_STA_CONNECTED BIT0
#define BIT_FTM_REPORT_OK BIT1
ESP_EVENT_DEFINE_BASE(PTP_FTM_CONTROL);
enum { FTM_CONTROL_START, FTM_CONTROL_CANCEL };
static ptp_ftm_session_t s_ftm_session;
static ptp_wifi_sta_timing_t s_ftm_timing;

/* 12.3/12.4 inputs: this radio initiates FTM, the peer's Extended Capabilities
 * come from the association record, and the grant follows the session result
 * for the frames-per-burst value actually requested. */
static portMUX_TYPE s_media_lock = portMUX_INITIALIZER_UNLOCKED;
static ptp_wifi_media_t s_ftm_media = {.ftm_local = true};
static uint8_t s_ftm_requested_now;

bool ptp_wifi_sta_media(int port_index, ptp_wifi_media_t *media) {
  if (!media || port_index < 0 || port_index != s_port_index || !s_events ||
      !(xEventGroupGetBits(s_events) & BIT_STA_CONNECTED)) return false;
  portENTER_CRITICAL(&s_media_lock);
  *media = s_ftm_media;
  portEXIT_CRITICAL(&s_media_lock);
  return true;
}

static void ftm_media_reset(void) {
  portENTER_CRITICAL(&s_media_lock);
  s_ftm_media = (ptp_wifi_media_t){.ftm_local = true};
  s_ftm_requested_now = 0;
  portEXIT_CRITICAL(&s_media_lock);
}
static int64_t s_ftm_associated_us;
static bool s_ftm_waiting_beacon;
static bool ftm_clock_beacon_fresh(void);

static void cancel_ftm_session(void) {
  ptp_ftm_session_invalidate(&s_ftm_session);
  if (s_ftm_session.pending) {
    esp_err_t result = esp_wifi_ftm_end_session();
    ESP_LOGI(TAG, "FTM association/timeout cancellation: %s",
             esp_err_to_name(result));
  }
}

/* Run on the same event loop as association changes and report consumption. */
static void on_ftm_control(void *arg, esp_event_base_t base, int32_t id,
                           void *data) {
  (void)arg; (void)base; (void)data;
  if (id == FTM_CONTROL_CANCEL) {
    cancel_ftm_session();
    return;
  }
  if (s_ftm_session.pending) return;
  wifi_ap_record_t ap_info = {0};
  if (!(xEventGroupGetBits(s_events) & BIT_STA_CONNECTED) ||
      esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
    xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
    return;
  }
  if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) {
    if (!ptpd_wifi_sta_timing_snapshot(s_port_index, &s_ftm_timing) ||
        s_ftm_timing.stopped || memcmp(s_ftm_timing.bssid, ap_info.bssid, 6)) {
      xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
      return;
    }
  }
  if (!s_ftm_associated_us) s_ftm_associated_us = esp_timer_get_time();
  if (!(ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) &&
      !ftm_clock_beacon_fresh()) {
    if (!s_ftm_waiting_beacon)
      ESP_LOGI(TAG, "FTM waiting for a fresh post-association clock beacon");
    s_ftm_waiting_beacon = true;
    xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
    return;
  }
  if (s_ftm_waiting_beacon) ESP_LOGI(TAG, "FTM clock beacon ready");
  s_ftm_waiting_beacon = false;
  wifi_ftm_initiator_cfg_t config = {
      .channel = ap_info.primary,
      .frm_count = FTM_FRM_COUNT,
      .burst_period = FTM_BURST_PERIOD_100MS,
  };
#ifdef WIFI_FTM_VENDOR_IE_MAX_LEN
  /* The vendor IE carries the Follow_Up (profiles/avb_lite.md, 802.1AS 12.5):
   * without the IDF FTM vendor IE API this is ranging only. */
  config.report_vendor_ie = ptp_ftm_report_hook != NULL;
#else
  static bool warned_no_vendor_ie;
  if (!warned_no_vendor_ie) {
    warned_no_vendor_ie = true;
    ESP_LOGW(TAG, "FTM vendor IE API not in this IDF; wireless time transfer unavailable, ranging only");
  }
#endif
  static uint8_t ftm_requested_frames = 3;
  if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) {
    config.burst_period = 0;
    config.frm_count = ftm_requested_frames;
  }
  memcpy(config.resp_mac, ap_info.bssid, 6);
  ptp_ftm_session_begin(&s_ftm_session, ap_info.bssid);
  if (ptp_ftm_begin_hook) ptp_ftm_begin_hook(ap_info.bssid);
  esp_err_t result = esp_wifi_ftm_initiate_session(&config);
  portENTER_CRITICAL(&s_media_lock);
  s_ftm_media.ftm_peer = ap_info.ftm_responder;
  s_ftm_requested_now = result == ESP_OK ? config.frm_count : 0;
  portEXIT_CRITICAL(&s_media_lock);
  if (result != ESP_OK) {
    if (result == ESP_ERR_INVALID_ARG && config.frm_count == 3) {
      ftm_requested_frames = 2;
      ESP_LOGW(TAG, "Three-frame FTM request rejected by SDK, testing two-frame request");
    } else if (result == ESP_ERR_INVALID_ARG && config.frm_count == 2) {
      ftm_requested_frames = 8;
      ESP_LOGW(TAG, "Two-frame FTM request rejected by SDK, testing eight-frame fallback");
    }
    s_ftm_session.pending = s_ftm_session.valid = false;
    ESP_LOGW(TAG, "esp_wifi_ftm_initiate_session: %s", esp_err_to_name(result));
    xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
  }
}

/* FTM-derived sync markers. on_vendor_ie writes both as IEs arrive;
 * the FTM_REPORT success handler combines them with FTM t1 to compute
 * BTC time at the FTM TX moment and injects via inject_sync_pair.
 * Both IEs ride the same beacon and are processed back-to-back, so
 * the pair is naturally atomic.
 *
 *   s_gptp_marker_ns: BTC time at bridge marshal moment (§12.7 IE
 *                     preciseOriginTimestamp).
 *   s_tsf_marker_us:  bridge AP TSF µs at coprocessor publish moment
 *                     (TSF mapping IE).
 *
 * Both must be non-zero before the FTM handler uses the pair. */
static int64_t s_gptp_marker_ns;
static int64_t s_tsf_marker_us;
static portMUX_TYPE s_tsf_marker_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_tsf_marker_received_us;
static bool s_seen_gptp;
static bool s_seen_tsf;

static bool ftm_clock_beacon_fresh(void) {
  int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_tsf_marker_lock);
  bool fresh = ptp_ftm_beacon_fresh(s_seen_tsf, s_seen_gptp,
      s_ftm_associated_us, s_tsf_marker_received_us, now_us);
  portEXIT_CRITICAL(&s_tsf_marker_lock);
  return fresh;
}

/* Latest FTM-derived link delay (ns). The beacon disciplining path adds
 * it to BTC_now; FTM only refines this value, so the clock still tracks
 * when FTM is unavailable (it just loses the ~propagation correction). */
static int64_t s_peer_delay_ns;

/* FTM counter continuity and independent beacon reboot evidence. */
static uint64_t s_prev_ftm_t1_ps;
static int64_t s_prev_beacon_tsf_us;
static bool s_have_ftm_baseline;
static uint32_t s_tsf_backward_streak;
#define TSF_BOUNCE_CONSEC_THRESHOLD 3

/* AVB Wireless station status (profiles/avb_wireless.md §2.6, §5.1).
 * Locked needs an element applied within three Sync intervals at log -3.
 * In Mode B the servo counts as converged once its error is within
 * BEACON_LOCK_ENTER_NS and stops once it exceeds BEACON_LOCK_EXIT_NS. */
#define STA_LOCK_WINDOW_US 375000
#define BEACON_LOCK_ENTER_NS 2000000
#define BEACON_LOCK_EXIT_NS 4000000
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_sta_associated_us;
static uint8_t s_status_bssid[6];
static int64_t s_beacon_applied_us;
static int32_t s_beacon_error_ns;
static bool s_beacon_error_valid, s_beacon_converged, s_beacon_was_locked;
static uint8_t s_beacon_port_identity[10];
static int32_t s_ftm_rtt_ns;
static bool s_ftm_rtt_valid;
static uint16_t s_ap_resets;
static ptp_ftm_success_t s_ftm_success;

static int32_t status_clamp_ns(int64_t value_ns) {
  if (value_ns > INT32_MAX) return INT32_MAX;
  if (value_ns < INT32_MIN) return INT32_MIN;
  return (int32_t)value_ns;
}

/* A new time source: the station acquires again before it is locked. */
static void beacon_status_new_source(void) {
  portENTER_CRITICAL(&s_status_lock);
  s_beacon_applied_us = 0;
  s_beacon_error_valid = s_beacon_converged = s_beacon_was_locked = false;
  memset(s_beacon_port_identity, 0, sizeof(s_beacon_port_identity));
  portEXIT_CRITICAL(&s_status_lock);
}

static void beacon_status_applied(int64_t error_ns) {
  int64_t magnitude_ns = error_ns < 0 ? -error_ns : error_ns;
  portENTER_CRITICAL(&s_status_lock);
  s_beacon_applied_us = esp_timer_get_time();
  s_beacon_error_ns = status_clamp_ns(error_ns);
  s_beacon_error_valid = true;
  if (magnitude_ns <= BEACON_LOCK_ENTER_NS) s_beacon_converged = true;
  else if (magnitude_ns > BEACON_LOCK_EXIT_NS) s_beacon_converged = false;
  if (s_beacon_converged) s_beacon_was_locked = true;
  portEXIT_CRITICAL(&s_status_lock);
}

static void ftm_status_measured(bool valid, int32_t rtt_ns) {
  portENTER_CRITICAL(&s_status_lock);
  ptp_ftm_success_record(&s_ftm_success, esp_timer_get_time(), valid);
  if (valid) {
    s_ftm_rtt_ns = rtt_ns;
    s_ftm_rtt_valid = true;
  }
  portEXIT_CRITICAL(&s_status_lock);
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  (void)arg;
  (void)base;
  switch (id) {
  case WIFI_EVENT_STA_CONNECTED: {
    const wifi_event_sta_connected_t *connected = data;
    if (memcmp(s_status_bssid, connected->bssid, 6)) {
      memcpy(s_status_bssid, connected->bssid, 6);
      beacon_status_new_source();
    }
    portENTER_CRITICAL(&s_status_lock);
    s_sta_associated_us = esp_timer_get_time();
    s_ftm_rtt_valid = false;
    portEXIT_CRITICAL(&s_status_lock);
    if (ptp_ftm_reset_hook) ptp_ftm_reset_hook();
    ftm_media_reset();
    cancel_ftm_session();
    s_ftm_associated_us = esp_timer_get_time();
    s_have_ftm_baseline = false;
    s_prev_beacon_tsf_us = 0;
    s_tsf_backward_streak = 0;
    xEventGroupSetBits(s_events, BIT_STA_CONNECTED);
    break;
  }
  case WIFI_EVENT_STA_STOP:
  case WIFI_EVENT_STA_DISCONNECTED:
    if (ptp_ftm_reset_hook) ptp_ftm_reset_hook();
    ftm_media_reset();
    cancel_ftm_session();
    if (id == WIFI_EVENT_STA_STOP) s_ftm_session.pending = false;
    s_ftm_associated_us = 0;
    s_have_ftm_baseline = false;
    s_prev_beacon_tsf_us = 0;
    s_tsf_backward_streak = 0;
    portENTER_CRITICAL(&s_status_lock);
    s_sta_associated_us = 0;
    portEXIT_CRITICAL(&s_status_lock);
    xEventGroupClearBits(s_events, BIT_STA_CONNECTED);
    break;
  case WIFI_EVENT_FTM_REPORT: {
    wifi_event_ftm_report_t *r = (wifi_event_ftm_report_t *)data;
    /* 12.4 c) 3): only a granted three- or two-frame request counts. The
     * eight-frame SDK fallback keeps experimental discipline running but
     * never qualifies the port. */
    portENTER_CRITICAL(&s_media_lock);
    bool declined = r->status == FTM_STATUS_CONF_REJECTED ||
        r->status == FTM_STATUS_UNSUPPORTED || r->status == FTM_STATUS_NO_RESPONSE;
    s_ftm_media.granted_frames = !declined &&
        (s_ftm_requested_now == 3 || s_ftm_requested_now == 2) ? s_ftm_requested_now : 0;
    portEXIT_CRITICAL(&s_media_lock);
    wifi_ap_record_t current_ap = {0};
    bool connected = (xEventGroupGetBits(s_events) & BIT_STA_CONNECTED) &&
        esp_wifi_sta_get_ap_info(&current_ap) == ESP_OK &&
        memcmp(current_ap.bssid, r->peer_mac, 6) == 0 &&
        ((ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) ||
         ftm_clock_beacon_fresh());
    if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) {
      ptp_wifi_sta_timing_t current;
      connected = connected && ptpd_wifi_sta_timing_snapshot(s_port_index, &current) &&
          !current.stopped && current.association == s_ftm_timing.association &&
          current.revision == s_ftm_timing.revision &&
          !memcmp(current.bssid, s_ftm_timing.bssid, 6);
    }
    if (!ptp_ftm_session_finish(&s_ftm_session, r->peer_mac, connected)) {
      if (r->status == FTM_STATUS_SUCCESS)
        (void)esp_wifi_ftm_get_report(NULL, 0);
#ifdef WIFI_FTM_VENDOR_IE_MAX_LEN
      (void)esp_wifi_ftm_get_vendor_data(NULL, 0);
#endif
      ESP_LOGI(TAG, "FTM report discarded across association/freshness/timeout boundary: status=%d",
               r->status);
      xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
      break;
    }
    if (ptp_ftm_report_hook && ptp_ftm_report_hook(r)) {
      xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
      break;
    }
    if (r->status == FTM_STATUS_SUCCESS) {
      /* Average only positive signed RTTs for the sub-ns fallback.
       * Small negative RTTs retain their two's-complement representation
       * in the unsigned driver field; zero and invalid sentinels also
       * cannot supply a delay or timestamp anchor. */
      uint64_t avg_rtt_ps = 0;
      uint8_t valid = 0;
      uint8_t n = r->ftm_report_num_entries;
      if (n > 16)
        n = 16;
      uint64_t best_t1_ps = 0;
      bool have_t1 = false;
      if (n) {
        wifi_ftm_report_entry_t entries[16];
        if (esp_wifi_ftm_get_report(entries, n) == ESP_OK) {
          uint64_t sum_ps = 0;
          int last_valid = -1;
          for (uint8_t i = 0; i < n; ++i) {
            if (ptp_ftm_rtt_positive(entries[i].rtt)) {
              sum_ps += entries[i].rtt;
              valid++;
              last_valid = i;
            }
          }
          if (valid) {
            avg_rtt_ps = sum_ps / valid;
          }
          if (last_valid >= 0) {
            best_t1_ps = entries[last_valid].t1;
            have_t1 = true;
          }
        }
      }

      /* A consumed report must wake the ranging task on every exit path. */
      xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
      if (have_t1) {
        int64_t now_us = esp_timer_get_time();
        portENTER_CRITICAL(&s_tsf_marker_lock);
        bool fresh_beacon = s_seen_tsf && s_tsf_marker_received_us > 0 &&
            now_us >= s_tsf_marker_received_us &&
            now_us - s_tsf_marker_received_us <= 1000000;
        int64_t beacon_tsf_us = fresh_beacon ? s_tsf_marker_us : 0;
        portEXIT_CRITICAL(&s_tsf_marker_lock);
        bool ftm_backward = s_have_ftm_baseline &&
            ptp_ftm_counter_backward(s_prev_ftm_t1_ps, best_t1_ps);
        bool beacon_backward = s_have_ftm_baseline && beacon_tsf_us > 0 &&
            s_prev_beacon_tsf_us > beacon_tsf_us &&
            s_prev_beacon_tsf_us - beacon_tsf_us > 1000000;
        if (beacon_backward) {
          if (++s_tsf_backward_streak < TSF_BOUNCE_CONSEC_THRESHOLD) {
            ESP_LOGW(TAG, "Beacon TSF moved backward, awaiting confirmation");
            ftm_status_measured(false, 0);
            break;
          }
          ESP_LOGW(TAG, "Beacon TSF reset confirmed, reassociating");
          ftm_status_measured(false, 0);
          portENTER_CRITICAL(&s_status_lock);
          if (s_ap_resets < UINT16_MAX) ++s_ap_resets;
          portEXIT_CRITICAL(&s_status_lock);
          beacon_status_new_source();
          s_tsf_backward_streak = 0;
          s_have_ftm_baseline = false;
          portENTER_CRITICAL(&s_tsf_marker_lock);
          s_seen_tsf = false;
          s_tsf_marker_received_us = 0;
          portEXIT_CRITICAL(&s_tsf_marker_lock);
          s_seen_gptp = false;
          esp_wifi_disconnect();
          break;
        }
        s_tsf_backward_streak = 0;
        if (ftm_backward) {
          static uint32_t bad_ftm_count;
          if ((++bad_ftm_count % 10) == 1)
            ESP_LOGW(TAG, "FTM counter moved backward without beacon reset");
          ftm_status_measured(false, 0);
          break;
        }
        s_prev_ftm_t1_ps = best_t1_ps;
        if (beacon_tsf_us > 0) s_prev_beacon_tsf_us = beacon_tsf_us;
        s_have_ftm_baseline = true;
      }
      /* Prefer the IDF's calibrated rtt_est: it is the noise-filtered
       * estimate (the basis for dist_est) and is what gPTP's
       * neighborPropDelay expects — single-digit-to-tens of ns at room
       * scale. The per-entry ps average is kept only as a close-range
       * fallback for when rtt_est truncates to 0 (sub-ns), a regime where
       * FTM ranging is unreliable anyway (RTT floors at <=0 and IDF
       * discards the entries). */
      int64_t peer_delay_ns = 0;
      bool have_delay = true;
      if (r->rtt_est) {
        peer_delay_ns = (int64_t)r->rtt_est / 2;
      } else if (avg_rtt_ps) {
        uint64_t one_way_ps = avg_rtt_ps / 2;
        peer_delay_ns = (int64_t)((one_way_ps + 500) / 1000);
      } else {
        have_delay = false;
      }
      int rc = have_delay
                   ? ptpd_inject_peer_delay(s_port_index, peer_delay_ns)
                   : -1;

      /* Adopt the link delay for the Layer-2 offset path (on_vendor_ie)
       * only if ptpd accepted it (rc == 0). ptpd rejects negative /
       * out-of-range samples with -ERANGE; reusing that one check means a
       * garbage reading — a multipath spike, or a point-blank ms-scale
       * per-entry average — cannot poison the beacon-paired offset. On
       * rejection, hold the last good delay. The BTC timeline itself is
       * carried by the beacon's (BTC, AP-TSF) pair + the STA TSF, so the
       * broken ToD weld stays gone. */
      if (rc == 0) {
        s_peer_delay_ns = peer_delay_ns;
      }
      int64_t rtt_ns = r->rtt_est ? (int64_t)r->rtt_est
                                  : (int64_t)((avg_rtt_ps + 500) / 1000);
      ftm_status_measured(rc == 0, status_clamp_ns(rtt_ns));

      static uint32_t s_seen = 0;
      if ((++s_seen % 25) == 1) {
        ESP_LOGI(TAG,
                 "FTM report #%u: peer %02x:%02x:%02x:%02x:%02x:%02x "
                 "RTT_est=%u ns  avg_rtt=%llu ps (%u/%u valid)  "
                 "peer_delay=%lld ns  inject_rc=%d",
                 (unsigned)s_seen, r->peer_mac[0], r->peer_mac[1],
                 r->peer_mac[2], r->peer_mac[3], r->peer_mac[4], r->peer_mac[5],
                 (unsigned)r->rtt_est, (unsigned long long)avg_rtt_ps, valid, n,
                 (long long)peer_delay_ns, rc);
      }
    } else {
      ftm_status_measured(false, 0);
      /* Failure path. In practice num_entries is always 0 here: IDF
       * receives the FTM action frames (it logs "N received measurements")
       * but yields zero valid report entries, so there are no per-entry
       * t1..t4 to inspect. The only initiator-side data IDF exposes on a
       * failed session are the aggregates below: rtt_raw/rtt_est are the
       * RTTs it computed before rejecting them (0 if it derived none) and
       * dist_est its one-way distance estimate. These show whether IDF
       * computed anything at all from the frames or discarded them
       * outright. We do NOT yet know which side is at fault (responder
       * t1/t4 vs local t2/t3 capture); the empty report hides it. */
      ESP_LOGW(TAG,
               "FTM session failed: status=%d num_entries=%u "
               "rtt_raw=%u ns rtt_est=%u ns dist_est=%u cm",
               r->status, r->ftm_report_num_entries, (unsigned)r->rtt_raw,
               (unsigned)r->rtt_est, (unsigned)r->dist_est);
      if (r->status == FTM_STATUS_NO_VALID_MSMT) {
        /* Proximity floor. This status means the session ran and the
         * responder answered (the driver logs the received measurement
         * count) but every computed RTT landed at or below zero, the
         * devices sit closer than FTM's turnaround-calibration floor
         * (roughly a metre, true RTT of single-digit ns). The real
         * propagation delay is below anything FTM can resolve here, so
         * inject zero rather than leaving neighborPropDelay undefined:
         * a floor-limited zero is accurate to within the measurement
         * resolution, and the peer-delay mechanism keeps producing the
         * value 802.1AS expects. Genuinely failed sessions
         * (UNSUPPORTED, NO_RESPONSE, FAIL) keep the no-injection path
         * below, there the peer may actually be gone. */
        int zrc = ptpd_inject_peer_delay(s_port_index, 0);
        if (zrc == 0) {
          s_peer_delay_ns = 0;
        }
        static uint32_t s_zero_injects = 0;
        if ((++s_zero_injects % 25) == 1) {
          ESP_LOGI(TAG,
                   "FTM below proximity floor (status=%d), peer delay "
                   "injected as 0 ns (count=%u, rc=%d)",
                   r->status, (unsigned)s_zero_injects, zrc);
        }
      }
      /* No reassociation on FTM failure — the STA still receives the FTM
       * frames (RSSI healthy, beacons + §12.7 Follow_Ups keep flowing), so
       * churning the link cannot help and (with the default STA netif
       * present) would re-attach the netif rxcb and stall AVB RX. The
       * genuine AP-reboot case is still caught by the TSF-backwards check
       * on the success path above. Time transfer falls back to §12.7
       * beacon-IE markers without FTM ranging. */
      /* Diagnostic dump of per-entry t1..t4 on rare statuses where
       * IDF still populates the report (e.g. NO_VALID_MSMT). Tells us
       * which side is shipping zero/garbage timestamps. Always dump
       * on the first 5 failures, then every 25th, so we capture the
       * initial bursts. */
      static uint32_t s_fail = 0;
      ++s_fail;
      if (r->ftm_report_num_entries && (s_fail <= 5 || s_fail % 25 == 0)) {
        wifi_ftm_report_entry_t entries[16];
        uint8_t n = r->ftm_report_num_entries;
        if (n > 16)
          n = 16;
        if (esp_wifi_ftm_get_report(entries, n) == ESP_OK) {
          for (uint8_t i = 0; i < n; ++i) {
            const wifi_ftm_report_entry_t *e = &entries[i];
            ESP_LOGW(TAG,
                     "  entry %u: rssi=%d rtt=%u ps t1=%llu t2=%llu "
                     "t3=%llu t4=%llu ppm=%d",
                     i, e->rssi, (unsigned)e->rtt, (unsigned long long)e->t1,
                     (unsigned long long)e->t2, (unsigned long long)e->t3,
                     (unsigned long long)e->t4, e->ppm);
          }
        }
      }
    }
    break;
  }
  default:
    break;
  }
}

/* Vendor IE callback — fires for every Vendor IE in scanned/received
 * beacons and probe responses. Filters to our OUI and decodes the
 * combined FOLLOWUP IE the bridge publishes: an IEEE 802.1AS-2020
 * §12.7 Follow_Up (its preciseOriginTimestamp is the BTC marker) with
 * the bridge AP TSF µs appended (the AP-TSF marker). Both are captured
 * for the same beacon, giving the STA an atomic (BTC, AP-TSF) anchor;
 * combined with the STA's own TSF (the same counter as the AP TSF) it
 * disciplines its clock to BTC. FTM supplies only the link delay. */
static void on_vendor_ie(void *ctx, wifi_vendor_ie_type_t type,
                         const uint8_t sa[6], const vendor_ie_data_t *vnd_ie,
                         int rssi) {
  (void)ctx;
  /* FTM carries its own source-qualified timing. Do not require or ingest
   * the private beacon mapping in this mode. */
  if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) return;
  if (type != WIFI_VND_IE_TYPE_BEACON) {
    return;
  }
  if (vnd_ie->vendor_oui[0] != PTP_VND_IE_OUI0 ||
      vnd_ie->vendor_oui[1] != PTP_VND_IE_OUI1 ||
      vnd_ie->vendor_oui[2] != PTP_VND_IE_OUI2) {
    return;
  }
  /* vnd_ie->length covers OUI(3) + oui_type(1) + payload. */
  int payload_len = (int)vnd_ie->length - 4;
  const uint8_t *payload = vnd_ie->payload;

  /* Only the combined FOLLOWUP IE is published now; the AP-TSF marker
   * rides its trailing bytes (parsed below) rather than a separate IE. */
  if (vnd_ie->vendor_oui_type != PTP_VND_IE_OUI_TYPE_FOLLOWUP) {
    return;
  }

  static uint32_t s_seen = 0;
  ++s_seen;

  /* Validate size: §12.7 Follow_Up payload + the appended AP-TSF marker
   * (the bridge combines them into one IE for atomic BTC/TSF pairing). */
  const int expect_len =
      (int)sizeof(struct ptp_follow_up_s) + PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN;
  if (payload_len != expect_len) {
    if ((s_seen % 50) == 1) {
      ESP_LOGW(TAG,
               "Beacon Vendor IE from %02x:%02x:%02x:%02x:%02x:%02x: "
               "payload %d B (expected %d for §12.7 FU + AP-TSF). Skipped.",
               sa[0], sa[1], sa[2], sa[3], sa[4], sa[5], payload_len,
               expect_len);
    }
    return;
  }

  const struct ptp_follow_up_s *fu = (const struct ptp_follow_up_s *)payload;
  const struct ptp_header_s *h = &fu->header;

  /* Sanity-check the messagetype nibble matches Follow_Up. In gPTP
   * the high nibble carries majorSdoId; mask it off before comparing. */
  if ((h->messagetype & PTP_MSGTYPE_MASK) != PTP_MSGTYPE_FOLLOW_UP) {
    if ((s_seen % 50) == 1) {
      ESP_LOGW(TAG,
               "Beacon Vendor IE messagetype 0x%02x not Follow_Up; "
               "skipped.",
               h->messagetype);
    }
    return;
  }

  /* Decode and log every Nth beacon. RX timestamp would come from the
   * radio for true §12 timing — esp_wifi_set_vendor_ie_cb doesn't
   * surface it, so we settle for the dedup + FTM pair-injection
   * pipeline instead (matches our chosen carrier-deviation tradeoff). */
  if ((s_seen % 50) == 1) {
    const uint8_t *gm = h->sourceidentity;
    uint16_t seq = ((uint16_t)h->sequenceid[0] << 8) | h->sequenceid[1];
    int64_t correction_ns = 0;
    for (int i = 0; i < 6; i++) {
      correction_ns = (correction_ns << 8) | h->correction[i];
    }
    uint64_t secs = 0;
    for (int i = 0; i < 6; i++) {
      secs = (secs << 8) | fu->origintimestamp[i];
    }
    uint32_t nsecs = ((uint32_t)fu->origintimestamp[6] << 24) |
                     ((uint32_t)fu->origintimestamp[7] << 16) |
                     ((uint32_t)fu->origintimestamp[8] << 8) |
                     (uint32_t)fu->origintimestamp[9];

    ESP_LOGI(TAG,
             "§12.7 Follow_Up @beacon from %02x:%02x:%02x:%02x:%02x:%02x "
             "RSSI=%d seen=%u: GM clockIdentity=%02x:%02x:%02x:%02x:%02x:%02x:"
             "%02x:%02x seqId=%u correction=%lld ns precTS=%llu.%09lu",
             sa[0], sa[1], sa[2], sa[3], sa[4], sa[5], rssi, (unsigned)s_seen,
             gm[0], gm[1], gm[2], gm[3], gm[4], gm[5], gm[6], gm[7],
             (unsigned)seq, (long long)correction_ns, (unsigned long long)secs,
             (unsigned long)nsecs);
  }

  /* Stash the §12.7 preciseOriginTimestamp as gptp_marker BEFORE
   * the dedup below — dedup gates downstream inject_sync (to avoid
   * aliasing the servo's rate estimate) but we want the marker to
   * track every beacon so FTM pair-injection always uses a current
   * pair. */
  {
    uint64_t secs = 0;
    for (int i = 0; i < 6; i++) {
      secs = (secs << 8) | fu->origintimestamp[i];
    }
    uint32_t nsecs = ((uint32_t)fu->origintimestamp[6] << 24) |
                     ((uint32_t)fu->origintimestamp[7] << 16) |
                     ((uint32_t)fu->origintimestamp[8] << 8) |
                     (uint32_t)fu->origintimestamp[9];
    s_gptp_marker_ns = (int64_t)(secs * 1000000000ULL) + (int64_t)nsecs;
    s_seen_gptp = true;
    /* The element's sourcePortIdentity is the AP's wireless port. */
    portENTER_CRITICAL(&s_status_lock);
    memcpy(s_beacon_port_identity, h->sourceidentity, 8);
    memcpy(&s_beacon_port_identity[8], h->sourceportindex, 2);
    portEXIT_CRITICAL(&s_status_lock);
  }

  /* The AP-TSF marker (µs, little-endian) is appended right after the
   * Follow_Up in the combined IE — atomically paired with the BTC marker
   * above (both captured for the same beacon on the bridge). */
  {
    const uint8_t *tsf = payload + sizeof(struct ptp_follow_up_s);
    int64_t tsf_us = 0;
    for (int i = 0; i < PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN; i++) {
      tsf_us |= ((int64_t)tsf[i]) << (8 * i);
    }
    int64_t received_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_tsf_marker_lock);
    s_tsf_marker_us = tsf_us;
    s_tsf_marker_received_us = received_us;
    s_seen_tsf = true;
    portEXIT_CRITICAL(&s_tsf_marker_lock);
  }

  /* Deduplicate against the previous-seen IE. Beacons fire every
   * ~100 ms (AP DTIM cadence) but the bridge only re-marshals the IE
   * every Sync interval (125 ms by default). So most beacons re-carry
   * the prior Sync's bytes; reprocessing them aliases the servo's
   * rate estimate. Skip when the preciseOriginTimestamp is unchanged
   * — that's the high-entropy field that always advances on a fresh
   * marshal. */
  static uint8_t s_last_origin_ts[10] = {0};
  if (memcmp(s_last_origin_ts, fu->origintimestamp, sizeof(s_last_origin_ts)) ==
      0) {
    return;
  }
  memcpy(s_last_origin_ts, fu->origintimestamp, sizeof(s_last_origin_ts));

  /* Discipline the clock to BTC in two layers, so the cross-chip pairing
   * gap can't corrupt the recovered rate:
   *   BTC_now = O_filt + sta_tsf_now*1000
   * Layer 1 (rate): sta_tsf_now*1000 is the AP-clock-locked timebase — the
   *   STA TSF is the same counter as the AP-TSF marker and is read locally
   *   with no cross-chip gap, so its rate is clean.
   * Layer 2 (offset): O = gptp_marker - tsf_mk*1000 + link_delay is the
   *   BTC<->AP-TSF offset. The BTC marker (host marshal) and the AP-TSF
   *   marker (coprocessor patch) are captured one SDIO RPC apart, so O
   *   carries that gap's variance. It is filtered separately (min-gap
   *   tracker, see below) so the gap never reaches the servo's integrator;
   *   the rate is unaffected because it rides the live STA TSF.
   * selected_source (GM identity/validity) comes from the §12.2 Announce,
   * which inject_sync_pair gates on. STA TSF and local clock are read
   * back-to-back so the injected pair is one instant. */
  if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled()) return;
  if (s_seen_gptp && s_seen_tsf) {
    int64_t sta_tsf_us = esp_wifi_get_tsf_time(WIFI_IF_STA);
    struct timespec swn = {0};
    ptpd_now(&swn);
    int64_t local_ns = (int64_t)swn.tv_sec * 1000000000LL + swn.tv_nsec;

    int64_t offset_raw =
        s_gptp_marker_ns - (int64_t)s_tsf_marker_us * 1000 + s_peer_delay_ns;

    /* Layer 2 recovers the BTC<->AP-TSF offset. The gap is always >= 0, so
     * O = true_offset - gap can only dip BELOW the truth; the cleanest
     * estimate is the smallest-gap (largest O) sample. So track the UPPER
     * ENVELOPE of O (a min-gap / minimum-delay filter, as PTP/NTP do for
     * asymmetric delay), not the mean — the mean sits a whole mean-gap
     * below the truth (tens of ms of constant offset error). Stages:
     *  - Acquire: over the first ACQ_SAMPLES, peak-hold the min-gap offset
     *    WITHOUT disciplining (clock free-runs); the first post-acquire
     *    inject is then one clean jump, not a long slew.
     *  - Track (peak-hold + slow decay): rise toward cleaner samples but
     *    cap the step below the servo slew so it never rails; decay slowly
     *    to follow the BTC<->AP-TSF crystal drift (ignoring how deep each
     *    gap dips); snap on a genuine step (large upward, or a sustained
     *    large-downward run that isn't just a transient gap spike). The
     *    rate rides the live STA TSF, so the residual offset is ~the floor
     *    (min gap) instead of the mean gap. */
    static int s_acq = 0;
    static int64_t s_off_filt;
    static int s_down_run = 0;
    const int ACQ_SAMPLES = 16;
    const int64_t OFF_STEP_NS = 100LL * 1000 * 1000;     /* up step => snap+jump */
    const int OFF_UP_SHIFT = 1;                          /* rise fast to floor */
    const int64_t OFF_UP_MAX_NS = 400000;               /* but <= ~slew/sample */
    const int64_t OFF_DECAY_NS = 100000;                /* ~100 ppm drift follow */
    const int64_t OFF_DOWN_STEP_NS = 350LL * 1000 * 1000; /* > worst gap spike */
    const int OFF_DOWN_RUN = 8;                          /* sustained => real step */
    int64_t off_err = offset_raw - s_off_filt;

    if (s_acq < ACQ_SAMPLES) {
      if (s_acq == 0 || off_err > 0) {
        s_off_filt = offset_raw; /* peak-hold the min-gap sample */
      }
      s_acq++;
      static uint32_t s_acq_log = 0;
      if ((++s_acq_log % 8) == 1) {
        ESP_LOGI(TAG, "BTC discipline: acquiring offset %d/%d", s_acq,
                 ACQ_SAMPLES);
      }
    } else {
      if (off_err > OFF_STEP_NS) {
        s_off_filt = offset_raw; /* large upward: real step / recovery, snap */
        s_down_run = 0;
      } else if (off_err > 0) {
        int64_t up = off_err >> OFF_UP_SHIFT;
        if (up > OFF_UP_MAX_NS) {
          up = OFF_UP_MAX_NS;
        }
        s_off_filt += up; /* rise toward the min-gap floor */
        s_down_run = 0;
      } else if (off_err < -OFF_DOWN_STEP_NS && ++s_down_run >= OFF_DOWN_RUN) {
        s_off_filt = offset_raw; /* sustained deep-down: genuine downward step */
        s_down_run = 0;
      } else {
        s_off_filt -= OFF_DECAY_NS; /* higher gap: ignore depth, slow decay */
        if (off_err >= -OFF_DOWN_STEP_NS) {
          s_down_run = 0;
        }
      }
      int64_t btc_now_ns = s_off_filt + sta_tsf_us * 1000;
      int64_t off_ns = btc_now_ns - local_ns;
      if (ptpd_inject_sync_pair(s_port_index, btc_now_ns, local_ns) == 0)
        beacon_status_applied(off_ns);

      static uint32_t s_disc_seen = 0;
      if ((++s_disc_seen % 25) == 1) {
        ESP_LOGI(TAG,
                 "BTC discipline: off=%lld ns gap=%lld ns link_delay=%lld ns",
                 (long long)off_ns, (long long)off_err,
                 (long long)s_peer_delay_ns);
      }
    }
  }
}

/* FTM client task — initiates one burst per cadence interval against
 * the associated AP. Per IEEE 802.1AS-2020 §12.1.2, the client drives
 * the FTM exchange and uses the t1..t4 timestamps to compute peer
 * delay (and, via the (gPTP, TSF) marker pair, to inject a sync
 * pair). */
static void ftm_cadence_wake(void *task) {
  xTaskNotifyGive((TaskHandle_t)task);
}

static void ftm_client_task(void *arg) {
  (void)arg;
  xEventGroupWaitBits(s_events, BIT_STA_CONNECTED, pdFALSE, pdTRUE,
                      portMAX_DELAY);
  ESP_LOGI(TAG, "FTM client starting");

  bool precise_cadence = ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled();
  esp_timer_handle_t cadence_timer = NULL;
  if (precise_cadence) {
    esp_timer_create_args_t timer_args = {
        .callback = ftm_cadence_wake,
        .arg = xTaskGetCurrentTaskHandle(),
        .name = "ftm_cadence",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &cadence_timer));
  }
  int64_t next_start_us = esp_timer_get_time();
  while (true) {
    if ((xEventGroupGetBits(s_events) & BIT_STA_CONNECTED) == 0) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (precise_cadence) {
      ptp_wifi_sta_timing_t policy;
      if (!ptpd_wifi_sta_timing_snapshot(s_port_index, &policy) || policy.stopped) {
        vTaskDelay(pdMS_TO_TICKS(50));
        next_start_us = esp_timer_get_time();
        continue;
      }
    }
    xEventGroupClearBits(s_events, BIT_FTM_REPORT_OK);
    esp_err_t result = esp_event_post(PTP_FTM_CONTROL, FTM_CONTROL_START,
                                      NULL, 0, portMAX_DELAY);
    if (result != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    EventBits_t completed = xEventGroupWaitBits(s_events, BIT_FTM_REPORT_OK,
        pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    if (!(completed & BIT_FTM_REPORT_OK))
      (void)esp_event_post(PTP_FTM_CONTROL, FTM_CONTROL_CANCEL,
                           NULL, 0, portMAX_DELAY);
    /* §12.8.2 cadence: sleep one burst period between sessions. */
    if (precise_cadence) {
      int64_t now_us = esp_timer_get_time();
      next_start_us += 125000;
      if (next_start_us <= now_us) next_start_us = now_us + 125000;
      ESP_ERROR_CHECK(esp_timer_start_once(cadence_timer, next_start_us - now_us));
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
      vTaskDelay(pdMS_TO_TICKS(FTM_BURST_PERIOD_100MS * 100));
    }
  }
}

int ptp_wifi_sta_start(int port_index) {
  if (s_port_index >= 0) {
    return 0; /* idempotent: already started */
  }
  s_port_index = port_index;

  s_events = xEventGroupCreate();
  if (!s_events) {
    s_port_index = -1;
    return -1;
  }

  /* Multiple handlers may register for the same WIFI_EVENT; ptp.c's
   * ptp_wifi_event_handler tracks link state on the same events. */
  esp_err_t r = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                           on_wifi_event, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "esp_event_handler_register failed: %s", esp_err_to_name(r));
    vEventGroupDelete(s_events);
    s_events = NULL;
    s_port_index = -1;
    return -1;
  }

  r = esp_event_handler_register(PTP_FTM_CONTROL, ESP_EVENT_ANY_ID,
                                  on_ftm_control, NULL);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "FTM control handler registration failed: %s", esp_err_to_name(r));
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event);
    vEventGroupDelete(s_events);
    s_events = NULL;
    s_port_index = -1;
    return -1;
  }

  /* Race: applications typically wait for STA_CONNECTED before calling
   * ptpd_start_port, so the event has already fired by the time our
   * handler registers and BIT_STA_CONNECTED would otherwise never get
   * set. Probe esp_wifi_sta_get_ap_info() once at start — if the STA
   * is already associated, seed the bit so the FTM task can begin its
   * burst loop. Subsequent disconnect/reconnect cycles are handled
   * normally by the registered handler. */
  wifi_ap_record_t ap_info = {0};
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    /* The association began earlier; its age counts from here. */
    memcpy(s_status_bssid, ap_info.bssid, 6);
    portENTER_CRITICAL(&s_status_lock);
    s_sta_associated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_status_lock);
    xEventGroupSetBits(s_events, BIT_STA_CONNECTED);
  }

  r = esp_wifi_set_vendor_ie_cb(on_vendor_ie, NULL);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "esp_wifi_set_vendor_ie_cb failed: %s", esp_err_to_name(r));
  }

  /* 0x88f7 RX is fed in by the application's Wi-Fi dispatcher via
   * the public ptp_inject_received_frame() API (see esp_ptp.h) —
   * no callback registration needed here. */

  if (xTaskCreate(ftm_client_task, "ftm_client", 4096, NULL, 5, NULL) !=
      pdPASS) {
    ESP_LOGW(TAG, "xTaskCreate ftm_client failed");
  }

  if (ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled())
    ESP_LOGI(TAG, "FTM discipline: private beacon timing disabled");
  ESP_LOGI(TAG, "Wi-Fi PTP transport started on port %d", port_index);
  return 0;
}

#ifdef CONFIG_ESP_PTP_INTERVAL_PROBE
typedef struct {
  int port_index;
  uint32_t generation;
  bool ap;
  uint8_t source_mac[6], source_port[10], destination[6], target_port[10];
} interval_probe_t;
static interval_probe_t s_interval_probe;
static bool s_interval_probe_started;

/* Independent wire encoder for the bench, including the actual target identity. */
static void interval_probe_frame(uint8_t frame[72], const interval_probe_t *probe,
                                 uint16_t sequence, int8_t interval, bool wrong_identity) {
  memset(frame, 0, 72);
  memcpy(frame, probe->destination, 6);
  memcpy(frame + 6, probe->source_mac, 6);
  frame[12] = 0x88; frame[13] = 0xf7;
  uint8_t *message = frame + 14;
  message[0] = 0x1c; message[1] = 0x12; message[3] = 58;
  message[4] = CONFIG_ESP_PTP_DOMAIN;
  memcpy(message + 20, probe->source_port, 10);
  if (wrong_identity) message[29] ^= 0x40;
  message[30] = sequence >> 8; message[31] = sequence;
  message[33] = 127;
  memcpy(message + 34, probe->target_port, 10);
  const uint8_t tlv[14] = {0x80,0,0,10,0,0x80,0xc2,0,0,5,0,0,0,0};
  memcpy(message + 44, tlv, sizeof(tlv));
  message[54] = (uint8_t)interval;
}

#ifdef CONFIG_ESP_PTP_SYNC_INTERVAL_PROBE
static void sync_interval_probe_frame(uint8_t frame[74], const interval_probe_t *probe,
                                      uint16_t sequence, int8_t interval, bool wrong_identity) {
  interval_probe_frame(frame, probe, sequence, interval, wrong_identity);
  uint8_t *message = frame + 14;
  message[3] = 60;
  const uint8_t tlv[16] = {0,3,0,12,0,0x80,0xc2,0,0,2,128,0,128,0,0,0};
  memcpy(message + 44, tlv, sizeof(tlv));
  message[55] = (uint8_t)interval;
}
#endif

static void interval_probe_task(void *argument) {
  (void)argument;
  const struct { int8_t interval; bool wrong_identity; unsigned wait_ms; } stages[] = {
#ifdef CONFIG_ESP_PTP_SYNC_INTERVAL_PROBE
    {127,true,4000}, {0,false,4000}, {127,false,5000}, {-128,false,3000},
    {126,false,15000}, {-3,false,3000}, {126,false,0}
#else
    {1,false,16000}, {-3,false,3000}, {127,true,2000}, {-4,false,2000},
    {127,false,3000}, {126,false,5000}, {126,false,0}
#endif
  };
#ifdef CONFIG_ESP_PTP_SYNC_INTERVAL_PROBE
  ESP_LOGI("cap_probe", "SYNCTEST_START,subtype2");
#endif
  vTaskDelay(pdMS_TO_TICKS(30000));
  for (unsigned index = 0; index < sizeof(stages) / sizeof(stages[0]); ++index) {
    if (s_interval_probe.generation != ptp_wifi_link_generation(s_interval_probe.port_index)) {
      ESP_LOGW("cap_probe", "CAPTEST_ABORT,association_changed");
      vTaskDelete(NULL);
      return;
    }
#ifdef CONFIG_ESP_PTP_SYNC_INTERVAL_PROBE
    uint8_t frame[74];
    sync_interval_probe_frame(frame, &s_interval_probe, index + 1,
                              stages[index].interval, stages[index].wrong_identity);
#else
    uint8_t frame[72];
    interval_probe_frame(frame, &s_interval_probe, index + 1,
                         stages[index].interval, stages[index].wrong_identity);
#endif
    int result = esp_wifi_internal_tx(s_interval_probe.ap ? 1 : 0, frame, sizeof(frame));
    ESP_LOGI("cap_probe", "CAPTEST_TX,%lld,%u,%d,%u,%d", esp_timer_get_time(),
             index + 1, stages[index].interval, stages[index].wrong_identity, result);
    if (stages[index].wait_ms) vTaskDelay(pdMS_TO_TICKS(stages[index].wait_ms));
  }
  ESP_LOGI("cap_probe", "CAPTEST_DONE");
  vTaskDelete(NULL);
}
#endif

void ptp_wifi_interval_probe_start(int port_index, bool ap, const uint8_t source_mac[6],
    const uint8_t source_port[10], const uint8_t destination[6],
    const uint8_t target_port[10], uint32_t generation) {
#ifdef CONFIG_ESP_PTP_INTERVAL_PROBE
  if (s_interval_probe_started) return;
  s_interval_probe = (interval_probe_t){.port_index = port_index, .generation = generation, .ap = ap};
  memcpy(s_interval_probe.source_mac, source_mac, 6);
  memcpy(s_interval_probe.source_port, source_port, 10);
  memcpy(s_interval_probe.destination, destination, 6);
  memcpy(s_interval_probe.target_port, target_port, 10);
  if (xTaskCreate(interval_probe_task, "cap_probe", 3072, NULL, 3, NULL) == pdPASS)
    s_interval_probe_started = true;
#else
  (void)port_index; (void)ap; (void)source_mac; (void)source_port;
  (void)destination; (void)target_port; (void)generation;
#endif
}

int ptpd_wifi_sta_status(int port_index, ptpd_wifi_sta_status_t *status) {
  if (!status || port_index < 0 || port_index != s_port_index || !s_events)
    return -EINVAL;
  memset(status, 0, sizeof(*status));
  int64_t now_us = esp_timer_get_time();
  bool mode_a = ptp_ftm_discipline_enabled && ptp_ftm_discipline_enabled();
  status->time_mode = mode_a ? ptp_wifi_time_mode_a_ftm : ptp_wifi_time_mode_b;
  status->associated = (xEventGroupGetBits(s_events) & BIT_STA_CONNECTED) != 0;

  int64_t applied_us = 0;
  uint8_t beacon_identity[10];
  portENTER_CRITICAL(&s_status_lock);
  if (!mode_a) {
    applied_us = s_beacon_applied_us;
    bool fresh = applied_us && now_us - applied_us <= STA_LOCK_WINDOW_US;
    status->locked = fresh && s_beacon_converged;
    status->holdover = !fresh && s_beacon_was_locked;
    status->servo_error_valid = s_beacon_error_valid;
    status->servo_error_ns = s_beacon_error_ns;
    status->rtt_valid = s_ftm_rtt_valid;
    status->rtt_ns = s_ftm_rtt_ns;
    status->ftm_success = ptp_ftm_success_percent(&s_ftm_success, now_us);
  }
  memcpy(beacon_identity, s_beacon_port_identity, sizeof(beacon_identity));
  status->ap_resets = s_ap_resets;
  if (s_sta_associated_us && status->associated)
    status->association_age_s = (uint32_t)((now_us - s_sta_associated_us) / 1000000);
  portEXIT_CRITICAL(&s_status_lock);

  if (mode_a) {
    ptp_ftm_status_t ftm = {.success = PTP_FTM_SUCCESS_NONE};
    if (ptp_ftm_status_hook && ptp_ftm_status_hook(&ftm)) {
      applied_us = ftm.applied_us;
      status->locked = ftm.locked;
      status->holdover = ftm.holdover;
      status->servo_error_valid = ftm.error_valid;
      status->servo_error_ns = ftm.error_ns;
      status->rtt_valid = ftm.rtt_valid;
      status->rtt_ns = ftm.rtt_ns;
    }
    status->ftm_success = ftm.success;
  }
  status->time_age_ms = PTP_WIFI_STATUS_AGE_NONE;
  if (applied_us && now_us >= applied_us &&
      (now_us - applied_us) / 1000 < PTP_WIFI_STATUS_AGE_NONE)
    status->time_age_ms = (uint16_t)((now_us - applied_us) / 1000);

  /* The SDK reports no grant on the initiator side, so only a three- or
   * two-frame request the responder accepted is known. */
  ptp_wifi_media_t media;
  if (ptp_wifi_sta_media(port_index, &media) && media.ftm_peer) {
    status->ftm_burst_frames = media.granted_frames ? media.granted_frames
                                                    : PTP_WIFI_STATUS_UNKNOWN;
    status->ftm_burst_duration = PTP_WIFI_STATUS_UNKNOWN;
    status->ftm_min_delta = PTP_WIFI_STATUS_UNKNOWN;
  } else {
    status->ftm_success = PTP_WIFI_STATUS_UNKNOWN;
  }

  uint8_t reason;
  ptp_wifi_sta_capability_status(port_index, status->ap_port_identity, &reason);
  status->as_capable_reason = mode_a ? reason : 0;
  static const uint8_t no_identity[10];
  if (!memcmp(status->ap_port_identity, no_identity, sizeof(no_identity)))
    memcpy(status->ap_port_identity, beacon_identity, sizeof(beacon_identity));
  return 0;
}
