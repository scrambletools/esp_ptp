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
#include "freertos/task.h"

#include "esp_ptp.h"
#include "ptp.h"
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

int ptp_wifi_ap_send_announce(int port_index, const uint8_t src_mac[6],
                              void *ptp_msg, uint16_t ptp_msg_len) {
  wifi_sta_list_t sta_list;
  memset(&sta_list, 0, sizeof(sta_list));
  esp_err_t r = esp_wifi_ap_get_sta_list(&sta_list);
  if (r != ESP_OK) {
    /* The wifi_remote bridge may not have an AP up yet, or the
     * coprocessor RPC isn't ready. Drop silently — the next tick
     * retries; no Announce will be missed once STAs associate. */
    return 0;
  }
  if (sta_list.num == 0) {
    return 0; /* No STAs to send to. */
  }

  int sent = 0;
  for (int i = 0; i < sta_list.num; i++) {
    int rc = wifi_ap_send_unicast_ptp(src_mac, sta_list.sta[i].mac, ptp_msg,
                                      ptp_msg_len);
    if (rc > 0)
      sent++;
  }

  static uint32_t s_seen = 0;
  if ((++s_seen % 30) == 1) {
    ESP_LOGI(TAG,
             "Sent unicast Announce to %d/%d associated STA(s) on port %d "
             "(seen=%u)",
             sent, sta_list.num, port_index, (unsigned)s_seen);
  }
  return sent;
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

static int s_port_index = -1;
static EventGroupHandle_t s_events;
#define BIT_STA_CONNECTED BIT0
#define BIT_FTM_REPORT_OK BIT1

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
static bool s_seen_gptp;
static bool s_seen_tsf;

/* AP-bounce detection. The AP's TSF (Timing Synchronization Function)
 * increments monotonically and resets to ~0 when the AP reboots. Each
 * FTM report carries the AP TSF at the FTM TX moment (per-entry t1 in
 * picoseconds). A sudden backward step in t1 across FTM cycles means
 * the AP rebooted while the STA was still associated — in that state
 * the STA's per-STA TX scheduler on the AP side stays corrupted until
 * the STA does a clean leave/rejoin (see Phase B in our investigation
 * trace). On detection, force esp_wifi_disconnect() so the application
 * Wi-Fi handler reconnects through the clean path. */
static uint64_t s_prev_ap_tsf_ps;
/* Backward step tolerance: ignore steps smaller than this — small
 * negative deltas can come from out-of-order packet processing or
 * clock corrections, not actual reboot. 1 s is well past any plausible
 * jitter and far below a real AP uptime. */
#define TSF_BOUNCE_THRESHOLD_PS (1000ULL * 1000ULL * 1000ULL * 1000ULL)

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id,
                          void *data) {
  (void)arg;
  (void)base;
  switch (id) {
  case WIFI_EVENT_STA_CONNECTED:
    xEventGroupSetBits(s_events, BIT_STA_CONNECTED);
    break;
  case WIFI_EVENT_STA_DISCONNECTED:
    xEventGroupClearBits(s_events, BIT_STA_CONNECTED);
    break;
  case WIFI_EVENT_FTM_REPORT: {
    wifi_event_ftm_report_t *r = (wifi_event_ftm_report_t *)data;
    if (r->status == FTM_STATUS_SUCCESS) {
      /* Per-entry rtt is picosecond-resolution; the aggregate rtt_est
       * is integer nanoseconds, which truncates to 0 at bench
       * distances where one-way delay is sub-ns. Average the valid
       * per-entry rtt values (zeros are invalid samples filtered by
       * IDF), halve for one-way, then round ps → ns at the daemon
       * boundary. */
      uint64_t avg_rtt_ps = 0;
      uint8_t valid = 0;
      uint8_t n = r->ftm_report_num_entries;
      if (n > 16)
        n = 16;
      uint64_t best_t1_ps = 0;
      uint64_t best_t2_ps = 0;
      if (n) {
        wifi_ftm_report_entry_t entries[16];
        if (esp_wifi_ftm_get_report(entries, n) == ESP_OK) {
          uint64_t sum_ps = 0;
          int last_valid = -1;
          for (uint8_t i = 0; i < n; ++i) {
            if (entries[i].rtt) {
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
            best_t2_ps = entries[last_valid].t2;
          }
        }
      }

      /* AP-bounce check: if AP TSF stepped backwards by more than the
       * tolerance, the AP rebooted while we were associated. Tear down
       * our (now-stale) association so the application reconnects from
       * scratch — this is the only way to get the AP's per-STA TX
       * scheduler back to a clean state when the underlying IDF wifi
       * AP-mode driver has the unicast-stuck bug. */
      if (best_t1_ps && s_prev_ap_tsf_ps) {
        if (best_t1_ps + TSF_BOUNCE_THRESHOLD_PS < s_prev_ap_tsf_ps) {
          ESP_LOGW(TAG,
                   "AP TSF stepped backwards (prev=%llu ps, now=%llu ps) — "
                   "AP rebooted; forcing clean reassociation",
                   (unsigned long long)s_prev_ap_tsf_ps,
                   (unsigned long long)best_t1_ps);
          /* Invalidate cached markers so we don't pair-inject against
           * the (now-meaningless) old bridge time anchor. */
          s_seen_tsf = false;
          s_seen_gptp = false;
          s_prev_ap_tsf_ps = 0;
          /* Fires WIFI_EVENT_STA_DISCONNECTED → app's handler calls
           * esp_wifi_connect() for a fresh Auth/Assoc handshake. */
          esp_wifi_disconnect();
          break; /* skip the rest of this report — markers are gone */
        }
      }
      if (best_t1_ps)
        s_prev_ap_tsf_ps = best_t1_ps;
      int64_t peer_delay_ns;
      if (avg_rtt_ps) {
        uint64_t one_way_ps = avg_rtt_ps / 2;
        peer_delay_ns = (int64_t)((one_way_ps + 500) / 1000);
      } else {
        /* No per-entry data — fall back to ns-resolution aggregate. */
        peer_delay_ns = (int64_t)r->rtt_est / 2;
      }
      int rc = ptpd_inject_peer_delay(s_port_index, peer_delay_ns);

      /* FTM-derived sync pair. With a fresh (gPTP, AP-TSF) mapping
       * AND a valid FTM measurement, convert the bridge-side hardware-
       * timestamped t1 (pSec on bridge TSF) into BTC time, then
       * back-project the STA local clock to the FTM RX moment via t2.
       * Pair-injection runs the same servo as the wired path. */
      int pair_rc = 0;
      int64_t t1_gPTP_ns = 0;
      int64_t local_at_RX_ns = 0;
      if (s_seen_gptp && s_seen_tsf && best_t1_ps && best_t2_ps) {
        int64_t t1_us = (int64_t)(best_t1_ps / 1000000ULL);
        int64_t delta_us = t1_us - s_tsf_marker_us;
        t1_gPTP_ns = s_gptp_marker_ns + delta_us * 1000;

        int64_t t2_us = (int64_t)(best_t2_ps / 1000000ULL);
        int64_t now_us = esp_timer_get_time();
        struct timespec swn = {0};
        ptpd_now(&swn);
        int64_t sw_now_ns = (int64_t)swn.tv_sec * 1000000000LL + swn.tv_nsec;
        local_at_RX_ns = sw_now_ns - (now_us - t2_us) * 1000;

        pair_rc =
            ptpd_inject_sync_pair(s_port_index, t1_gPTP_ns, local_at_RX_ns);
      }

      static uint32_t s_seen = 0;
      if ((++s_seen % 25) == 1) {
        ESP_LOGI(TAG,
                 "FTM report #%u: peer %02x:%02x:%02x:%02x:%02x:%02x "
                 "RTT_est=%u ns  avg_rtt=%llu ps (%u/%u valid)  "
                 "peer_delay=%lld ns  inject_rc=%d  "
                 "pair_rc=%d t1_gPTP=%lld local_RX=%lld",
                 (unsigned)s_seen, r->peer_mac[0], r->peer_mac[1],
                 r->peer_mac[2], r->peer_mac[3], r->peer_mac[4], r->peer_mac[5],
                 (unsigned)r->rtt_est, (unsigned long long)avg_rtt_ps, valid, n,
                 (long long)peer_delay_ns, rc, pair_rc, (long long)t1_gPTP_ns,
                 (long long)local_at_RX_ns);
      }
      xEventGroupSetBits(s_events, BIT_FTM_REPORT_OK);
    } else {
      ESP_LOGW(TAG, "FTM session failed: status=%d num_entries=%u", r->status,
               r->ftm_report_num_entries);
      /* No reassociation on FTM failure. status=5 (NO_VALID_MSMT) here
       * means the responder shipped unusable ranging timestamps, not
       * that frames went missing — the STA receives the FTM frames every
       * session (RSSI healthy, beacons + §12.7 Follow_Ups keep flowing)
       * but the driver rejects the measurements. A reassociation cannot
       * fix bad timestamps; it only churns the link, which (with the
       * default STA netif present) re-attaches the netif rxcb and stalls
       * AVB RX. The genuine AP-reboot case is still caught by the
       * TSF-backwards check on the success path above. Time transfer
       * falls back to §12.7 beacon-IE markers without FTM ranging. */
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
 * beacons and probe responses. Filters to our OUI and decodes both
 * IE sub-types we publish from the bridge:
 *
 *   FOLLOWUP    — IEEE 802.1AS-2020 §12.7 Follow_Up message. The
 *                  preciseOriginTimestamp is stashed as the BTC marker
 *                  for FTM pair-injection, and the parsed bytes are
 *                  fed into ptpd's selected_source via inject_sync.
 *   TSF_MAPPING — bridge AP TSF µs at coprocessor publish moment
 *                  (Scramble Tools-private). Paired with the §12.7
 *                  preciseOriginTimestamp it gives the STA a
 *                  (BTC, TSF) anchor that converts FTM t1 (TSF µs)
 *                  to BTC time. */
static void on_vendor_ie(void *ctx, wifi_vendor_ie_type_t type,
                         const uint8_t sa[6], const vendor_ie_data_t *vnd_ie,
                         int rssi) {
  (void)ctx;
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

  if (vnd_ie->vendor_oui_type == PTP_VND_IE_OUI_TYPE_TSF_MAPPING) {
    if (payload_len < PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN) {
      return;
    }
    int64_t tsf_us = 0;
    for (int i = 0; i < 8; ++i) {
      tsf_us |= ((int64_t)payload[i]) << (8 * i);
    }
    s_tsf_marker_us = tsf_us;
    s_seen_tsf = true;
    static uint32_t s_tsf_ie_seen = 0;
    if ((++s_tsf_ie_seen % 25) == 1) {
      ESP_LOGI(TAG, "TSF mapping IE #%u: ap_tsf=%lld us",
               (unsigned)s_tsf_ie_seen, (long long)tsf_us);
    }
    return;
  }

  if (vnd_ie->vendor_oui_type != PTP_VND_IE_OUI_TYPE_FOLLOWUP) {
    return;
  }

  static uint32_t s_seen = 0;
  ++s_seen;

  /* Validate size against an 802.1AS-2020 §12.7 Follow_Up payload. */
  if (payload_len != (int)sizeof(struct ptp_follow_up_s)) {
    if ((s_seen % 50) == 1) {
      ESP_LOGW(TAG,
               "Beacon Vendor IE from %02x:%02x:%02x:%02x:%02x:%02x: "
               "payload %d B (expected %u for §12.7 Follow_Up). Skipped.",
               sa[0], sa[1], sa[2], sa[3], sa[4], sa[5], payload_len,
               (unsigned)sizeof(struct ptp_follow_up_s));
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

  /* Feed Follow_Up into the daemon. inject_sync bootstraps —
   * sets selected_source (so inject_sync_pair has the BTC identity
   * it gates on) and jumps the SW clock to BTC time on the first
   * beacon. After that the FTM-derived pair injection (sub-ms
   * precision) runs alongside this coarse beacon-IE injection;
   * the servo converges to the FTM-precision regime. */
  (void)ptpd_inject_sync(s_port_index, payload, (size_t)payload_len);
}

/* FTM client task — initiates one burst per cadence interval against
 * the associated AP. Per IEEE 802.1AS-2020 §12.1.2, the client drives
 * the FTM exchange and uses the t1..t4 timestamps to compute peer
 * delay (and, via the (gPTP, TSF) marker pair, to inject a sync
 * pair). */
static void ftm_client_task(void *arg) {
  (void)arg;
  xEventGroupWaitBits(s_events, BIT_STA_CONNECTED, pdFALSE, pdTRUE,
                      portMAX_DELAY);
  ESP_LOGI(TAG, "FTM client starting");

  while (true) {
    if ((xEventGroupGetBits(s_events) & BIT_STA_CONNECTED) == 0) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    wifi_ap_record_t ap_info = {0};
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    static bool s_logged_ap_caps = false;
    if (!s_logged_ap_caps) {
      ESP_LOGI(TAG,
               "AP %02x:%02x:%02x:%02x:%02x:%02x ch=%u "
               "ftm_responder=%d (advertised in beacon)",
               ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
               ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5],
               ap_info.primary, ap_info.ftm_responder);
      s_logged_ap_caps = true;
    }
    wifi_ftm_initiator_cfg_t cfg = {
        .channel = ap_info.primary,
        .frm_count = FTM_FRM_COUNT,
        .burst_period = FTM_BURST_PERIOD_100MS,
    };
    memcpy(cfg.resp_mac, ap_info.bssid, 6);
    esp_err_t r = esp_wifi_ftm_initiate_session(&cfg);
    if (r != ESP_OK) {
      ESP_LOGW(TAG, "esp_wifi_ftm_initiate_session: %s", esp_err_to_name(r));
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    xEventGroupWaitBits(s_events, BIT_FTM_REPORT_OK, pdTRUE, pdFALSE,
                        pdMS_TO_TICKS(2000));
    /* §12.8.2 cadence: sleep one burst period between sessions. */
    vTaskDelay(pdMS_TO_TICKS(FTM_BURST_PERIOD_100MS * 100));
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
  if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "esp_event_handler_register failed: %s", esp_err_to_name(r));
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

  ESP_LOGI(TAG, "Wi-Fi PTP transport started on port %d", port_index);
  return 0;
}
