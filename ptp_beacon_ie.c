/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Host-side beacon Vendor IE publisher for gPTP FollowUpInformation
 * on a wifi_ftm port operating as AP. The daemon's TX loop drives the
 * publish cadence: on each Sync interval it invokes the registered
 * sync_egress_cb with the marshalled FollowUpInformation bytes; this
 * callback packs them into the Vendor IE and ships them to the
 * coprocessor via the esp_ptp_rpc custom-RPC channel for the radio's
 * esp_wifi_set_vendor_ie() to consume.
 *
 * The IEEE 802.1AS-2020 §12.7 FollowUpInformation TLV lives in the
 * 802.11 Beacon Vendor IE rather than in an FTM action-frame Vendor
 * IE (the latter is not reachable through the public ESP-IDF Wi-Fi
 * API). The byte layout inside the Vendor IE matches §12.7 verbatim
 * so a future carrier swap to FTM frames is wire-compatible.
 *
 * Why this lives in esp_ptp. The wire payload IS gPTP. The on-radio
 * carrier is just transport, like Pdelay frames on Ethernet. Owning
 * the publisher here keeps gPTP-state ↔ on-air-transport in one
 * component and lets the host application stay transport-agnostic.
 *
 * Why a custom RPC. esp_wifi_set_vendor_ie() linked locally on the
 * host is a silent no-op — the host-side esp_wifi_remote stub is
 * commented out in upstream esp-hosted-mcu and the host's local
 * libnet80211 has no radio. We use the additive Custom RPC channel
 * (esp_hosted_send_custom_data) instead; the coprocessor's
 * esp_ptp_rpc handler unpacks the buffer and calls
 * esp_wifi_set_vendor_ie() locally on the coprocessor where the
 * radio actually lives. See esp_ptp_rpc/src/ptp_custom_rpc.c.
 */

#include "sdkconfig.h"

#ifdef CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR

#include "esp_ptp.h"
#include "ptp_rpc_proto.h"

#include <stddef.h>
#include <string.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"

/* Forward declarations of the host-side ESP-Hosted custom-RPC API.
 * Declared in espressif__esp_hosted's esp_hosted_misc.h, which we
 * intentionally don't #include here so esp_ptp does not gain a hard
 * dependency on espressif__esp_hosted (projects without an ESP-Hosted
 * coprocessor won't have that component). The symbols only need to
 * resolve at link time for builds that actually use this code path
 * (CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR=y), and those builds always link
 * espressif__esp_hosted from elsewhere. */
extern esp_err_t esp_hosted_send_custom_data(uint32_t msg_id,
                                             const uint8_t *data,
                                             size_t data_len);
extern esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id,
    void (*cb)(uint32_t, const uint8_t *, size_t, void *),
    void *user);

static const char *TAG = "ptp_beacon_ie";

/* OUI: locally-administered placeholder (02:00:00) until Scramble
 * Tools obtains a registered OUI. Type field follows §12.7 Table 12-4
 * (0 = FollowUpInformation). */
#define AVB_VENDOR_IE_OUI0     0x02
#define AVB_VENDOR_IE_OUI1     0x00
#define AVB_VENDOR_IE_OUI2     0x00
#define AVB_VENDOR_IE_OUI_TYPE 0x00 /* §12.7 Type 0 = FollowUpInformation */

/* Which port is the wifi_ftm AP we publish for. CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR
 * gates this whole file, so exactly one of these branches matches in any
 * build that compiles us in. */
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
  /* Successful ACK is logged once at startup (see s_acked_at_least_once
   * tracking in the egress callback) — we don't spam INFO every Sync
   * interval. */
}

/* Tracking flag for one-shot startup log: only INFO-log the first
 * successful ACK; thereafter the publish is silent. Errors still
 * log every time. */
static bool s_acked_at_least_once = false;

/* Sync egress callback fired by ptp_periodic_send on every Sync
 * interval for our wifi_ftm AP port. fu_info points at the daemon-
 * marshalled FollowUpInformation bytes; we pack them into a Vendor
 * IE and dispatch to the coprocessor. */
static void on_sync_egress(int port_index, FAR const uint8_t *fu_info,
                           size_t fu_info_len, FAR void *ctx) {
  (void)port_index;
  (void)ctx;

  /* Wire frame: 4-byte ptp_rpc_set_vendor_ie_t header followed by the
   * vendor_ie_data_t buffer the coprocessor will hand to
   * esp_wifi_set_vendor_ie(). Vendor IE layout per 802.11 9.4.2.25 +
   * 802.1AS §12.7 Figure 12-8:
   *   element_id (1) | length (1) | OUI (3) | OUI type (1) | payload
   * The struct mapped onto this is `vendor_ie_data_t` in
   * esp_wifi_types_generic.h — first 6 bytes match. Stack-allocated;
   * the framework copies before transport. */
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
  ie[2] = AVB_VENDOR_IE_OUI0;
  ie[3] = AVB_VENDOR_IE_OUI1;
  ie[4] = AVB_VENDOR_IE_OUI2;
  ie[5] = AVB_VENDOR_IE_OUI_TYPE;
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
             PTP_BEACON_IE_PORT, AVB_VENDOR_IE_OUI0, AVB_VENDOR_IE_OUI1,
             AVB_VENDOR_IE_OUI2, AVB_VENDOR_IE_OUI_TYPE, fu_info_len);
    s_acked_at_least_once = true;
  }
}

/* Fires when the coprocessor's SoftAP comes up. At this moment
 * esp_wifi_set_vendor_ie() will actually take effect on the radio
 * (firing earlier would race against AP-mode init), AND s_state is
 * guaranteed non-NULL because the bridge waits for ptpd to be up
 * before bringing the AP up. Register the sync_egress_cb here. The
 * daemon's per-port loop already skips ports without an enabled
 * flag, so it's safe to register before ptpd_start_port(port=AP)
 * has been called — the cb won't fire until the port is enabled. */
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

/* Self-init at startup. Constructor runs before app_main while the
 * heap and log layer are live but before the FreeRTOS scheduler. We:
 *
 *   1. Ensure the default event loop exists. esp_event_loop_create_default
 *      is idempotent (returns ESP_ERR_INVALID_STATE if already created),
 *      so the host's later call from its own init path is a no-op.
 *
 *   2. Register on_wifi_ap_start for WIFI_EVENT_AP_START. The host
 *      will create+start the SoftAP somewhere in app_main; when that
 *      event fires our handler registers the sync_egress_cb.
 *
 *   3. Register the SET_VENDOR_IE_ACK callback so the coprocessor's
 *      reply isn't dropped as "no handler". Safe to call early —
 *      esp_hosted_register_custom_callback lazy-initializes its mutex
 *      and the dispatch table is BSS-allocated. */
__attribute__((constructor)) static void ptp_beacon_ie_autoreg(void) {
  (void)esp_event_loop_create_default();
  (void)esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START,
                                   on_wifi_ap_start, NULL);
  (void)esp_hosted_register_custom_callback(PTP_RPC_MSG_SET_VENDOR_IE_ACK,
                                            on_set_vendor_ie_ack, NULL);
}

#endif /* CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR */
