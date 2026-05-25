/* SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Beacon Vendor IE publisher for gPTP FollowUpInformation on a
 * wifi_ftm AP port. The byte layout matches 802.1AS-2020 §12.7 so
 * a future swap to FTM action frames is wire-compatible.
 *
 * Dispatched via esp_hosted custom RPC because the host's local
 * esp_wifi_set_vendor_ie() is a no-op (no radio on this side);
 * esp_ptp_rpc on the coprocessor calls it locally where the radio
 * actually lives.
 */

#include "sdkconfig.h"

#ifdef CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR

#include "esp_ptp.h"
#include "ptp.h"
#include "ptp_rpc_proto.h"

#include <stddef.h>
#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"

/* Forward-declared instead of #included so we don't pull in a hard
 * dependency on espressif__esp_hosted; symbols only need to resolve
 * in builds gated by CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR. */
extern esp_err_t esp_hosted_send_custom_data(uint32_t msg_id,
                                             const uint8_t *data,
                                             size_t data_len);
extern esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*cb)(uint32_t, const uint8_t *, size_t, void *),
    void *user);

static const char *TAG = "ptp_beacon_ie";

/* OUI / sub-types are shared with the coprocessor RPC handler and
 * STA parser via ptp_rpc_proto.h. */

#if defined(CONFIG_ESP_PTP_PORT0_MEDIUM_WIFI_FTM) && \
    defined(CONFIG_ESP_PTP_PORT0_WIFI_MODE_AP)
#define PTP_BEACON_IE_PORT 0
#elif defined(CONFIG_ESP_PTP_PORT1_MEDIUM_WIFI_FTM) && \
    defined(CONFIG_ESP_PTP_PORT1_WIFI_MODE_AP)
#define PTP_BEACON_IE_PORT 1
#else
#error "CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR set but no wifi_ftm+AP port found"
#endif

static void on_set_vendor_ie_ack(uint32_t msg_id, const uint8_t *data,
                                 size_t data_len, void *user) {
  (void)msg_id;
  (void)user;
  if (data_len < sizeof(ptp_rpc_set_vendor_ie_ack_t)) {
    ESP_LOGW(TAG, "SET_VENDOR_IE_ACK payload too small (%zu)", data_len);
    return;
  }
  const ptp_rpc_set_vendor_ie_ack_t *ack =
      (const ptp_rpc_set_vendor_ie_ack_t *)data;
  if (ack->esp_err != 0) {
    ESP_LOGE(TAG, "ACK: coprocessor returned %ld", (long)ack->esp_err);
  }
  /* Successful ACKs are logged once at startup; errors log every time. */
}

static bool s_acked_at_least_once = false;

static void on_sync_egress(int port_index, FAR const uint8_t *fu_info,
                           size_t fu_info_len, FAR void *ctx) {
  (void)port_index;
  (void)ctx;

  /* Wire frame: ptp_rpc_set_vendor_ie_t header + vendor_ie_data_t.
   * Vendor IE layout per 802.11 9.4.2.25 + 802.1AS §12.7 Figure 12-8. */
  uint8_t buf[sizeof(ptp_rpc_set_vendor_ie_t) + 6 + fu_info_len];
  memset(buf, 0, sizeof(buf));

  ptp_rpc_set_vendor_ie_t *hdr = (ptp_rpc_set_vendor_ie_t *)buf;
  hdr->enable = 1;
  hdr->type = WIFI_VND_IE_TYPE_BEACON;
  hdr->idx = WIFI_VND_IE_ID_0;
  hdr->reserved = 0;

  uint8_t *ie = buf + sizeof(*hdr);
  ie[0] = WIFI_VENDOR_IE_ELEMENT_ID; /* 0xDD */
  ie[1] = (uint8_t)(4 + fu_info_len);
  ie[2] = PTP_VND_IE_OUI0;
  ie[3] = PTP_VND_IE_OUI1;
  ie[4] = PTP_VND_IE_OUI2;
  ie[5] = PTP_VND_IE_OUI_TYPE_FOLLOWUP;
  if (fu_info != NULL && fu_info_len > 0) {
    memcpy(ie + 6, fu_info, fu_info_len);
  }

  esp_err_t r = esp_hosted_send_custom_data(PTP_RPC_MSG_SET_VENDOR_IE_REQ,
                                            buf, sizeof(buf));
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "esp_hosted_send_custom_data(SET_VENDOR_IE_REQ): %s",
             esp_err_to_name(r));
  } else if (!s_acked_at_least_once) {
    ESP_LOGI(TAG,
             "Beacon Vendor IE publish dispatched on port %d "
             "(OUI %02x:%02x:%02x type %d, payload %zu B). Subsequent "
             "publishes are silent unless an error occurs.",
             PTP_BEACON_IE_PORT, PTP_VND_IE_OUI0, PTP_VND_IE_OUI1,
             PTP_VND_IE_OUI2, PTP_VND_IE_OUI_TYPE_FOLLOWUP, fu_info_len);
    s_acked_at_least_once = true;
  }

  /* Companion (gPTP, AP-TSF) mapping IE in slot 1. TSF payload bytes
   * are placeholder zeros; the coprocessor RPC handler patches in the
   * live esp_wifi_get_tsf_time(WIFI_IF_AP) value before publishing.
   * STA pairs this with the §12.7 preciseOriginTimestamp to convert
   * FTM t1 into BTC time. */
  uint8_t tbuf[sizeof(ptp_rpc_set_vendor_ie_t) + 6 +
               PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN];
  memset(tbuf, 0, sizeof(tbuf));
  ptp_rpc_set_vendor_ie_t *thdr = (ptp_rpc_set_vendor_ie_t *)tbuf;
  thdr->enable = 1;
  thdr->type = WIFI_VND_IE_TYPE_BEACON;
  thdr->idx = WIFI_VND_IE_ID_1;
  thdr->reserved = 0;
  uint8_t *tie = tbuf + sizeof(*thdr);
  tie[0] = WIFI_VENDOR_IE_ELEMENT_ID;
  tie[1] = (uint8_t)(4 + PTP_VND_IE_TSF_MAPPING_PAYLOAD_LEN);
  tie[2] = PTP_VND_IE_OUI0;
  tie[3] = PTP_VND_IE_OUI1;
  tie[4] = PTP_VND_IE_OUI2;
  tie[5] = PTP_VND_IE_OUI_TYPE_TSF_MAPPING;
  /* tie[6..13] left zero; coprocessor patches in TSF µs LE. */
  esp_err_t r2 = esp_hosted_send_custom_data(PTP_RPC_MSG_SET_VENDOR_IE_REQ,
                                             tbuf, sizeof(tbuf));
  if (r2 != ESP_OK) {
    ESP_LOGE(TAG, "TSF mapping IE dispatch: %s", esp_err_to_name(r2));
  }
}

/* Fires when the coprocessor's SoftAP is up — earliest moment
 * esp_wifi_set_vendor_ie() takes effect on the radio. Safe to
 * register the egress cb before ptpd_start_port(port=AP) since
 * disabled ports skip dispatch. */
static void on_wifi_ap_start(void *arg, esp_event_base_t base, int32_t id,
                             void *event_data) {
  (void)arg;
  (void)base;
  (void)id;
  (void)event_data;
  int rc = ptpd_register_sync_egress_cb(PTP_BEACON_IE_PORT, on_sync_egress,
                                        NULL);
  if (rc != 0) {
    ESP_LOGE(TAG,
             "ptpd_register_sync_egress_cb(port=%d) failed: %d. Beacon "
             "IE publishes will not fire — has ptpd been started?",
             PTP_BEACON_IE_PORT, rc);
  } else {
    ESP_LOGI(TAG,
             "Registered sync_egress_cb for wifi_ftm AP on port %d; "
             "the daemon will dispatch FollowUpInformation at the "
             "configured gPTP Sync interval.",
             PTP_BEACON_IE_PORT);
  }
}

/* Constructor runs before app_main (heap + log layer live, scheduler
 * not yet running). Ensures the default event loop exists (idempotent),
 * arms the AP_START handler, and registers the ACK callback so the
 * coprocessor's reply isn't dropped. */
__attribute__((constructor)) static void ptp_beacon_ie_autoreg(void) {
  (void)esp_event_loop_create_default();
  (void)esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                   on_wifi_ap_start, NULL);
  (void)esp_hosted_register_custom_callback(PTP_RPC_MSG_SET_VENDOR_IE_ACK,
                                            on_set_vendor_ie_ack, NULL);
}

#endif /* CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR */
