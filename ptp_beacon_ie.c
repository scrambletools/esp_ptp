/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Host-side beacon Vendor IE publisher for gPTP FollowUpInformation
 * over a wifi_cp port operating as AP. Self-firing: hooked to
 * WIFI_EVENT_AP_START so the publish dispatches automatically the
 * moment the SoftAP comes up. No public API — fully internal to
 * esp_ptp.
 *
 * Path C of the AVB-over-Wi-Fi plan delivers the IEEE 802.1AS-2020
 * §12.7 FollowUpInformation TLV in the AP's 802.11 Beacon Vendor IE
 * rather than in the FTM action frame's Vendor IE (the latter is not
 * reachable through the public ESP-IDF Wi-Fi API). The byte layout
 * inside the Vendor IE matches §12.7 verbatim so the carrier swap to
 * FTM frames (CONFIG_ESP_AVB_FTM_IE_HOOK) is wire-compatible whenever
 * Espressif exposes that path.
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
 *
 * Current stub publishes a zero-payload FollowUpInformation. The
 * payload will eventually be marshalled live from ptpd's Sync stream
 * and this becomes a periodic publisher task.
 */

#include "sdkconfig.h"

#ifdef CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR

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
/* FollowUpInformation TLV byte length per §11.4.4 / §12.7. The full
 * Sync Follow_Up message is packed verbatim minus the 802.1AS frame
 * header; for the stub the bytes are all zero. */
#define AVB_FOLLOWUP_INFO_LEN  44

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
  if (ack->esp_err == 0) {
    ESP_LOGI(TAG, "ACK: coprocessor esp_wifi_set_vendor_ie ok");
  } else {
    ESP_LOGE(TAG, "ACK: coprocessor returned %ld", (long)ack->esp_err);
  }
}

static esp_err_t publish_followup_beacon_ie_stub(void) {
  /* Wire frame: 4-byte ptp_rpc_set_vendor_ie_t header followed by
   * the full vendor_ie_data_t buffer the coprocessor will hand to
   * esp_wifi_set_vendor_ie(). Stack-allocated; the framework copies
   * before transport. */
  uint8_t buf[sizeof(ptp_rpc_set_vendor_ie_t) + 6 + AVB_FOLLOWUP_INFO_LEN] = {0};
  ptp_rpc_set_vendor_ie_t *hdr = (ptp_rpc_set_vendor_ie_t *)buf;
  hdr->enable = 1;
  hdr->type = WIFI_VND_IE_TYPE_BEACON;
  hdr->idx = WIFI_VND_IE_ID_0;
  hdr->reserved = 0;

  /* Vendor IE byte layout per IEEE 802.11 9.4.2.25 plus our OUI/type
   * /payload from §12.7 Figure 12-8:
   *   element_id (1) | length (1) | OUI (3) | OUI type (1) | payload
   * The struct mapped onto this is `vendor_ie_data_t` in
   * esp_wifi_types_generic.h — first 6 bytes match. */
  uint8_t *ie = buf + sizeof(*hdr);
  ie[0] = WIFI_VENDOR_IE_ELEMENT_ID;        /* 0xDD */
  ie[1] = 4 + AVB_FOLLOWUP_INFO_LEN;        /* length per the spec field */
  ie[2] = AVB_VENDOR_IE_OUI0;
  ie[3] = AVB_VENDOR_IE_OUI1;
  ie[4] = AVB_VENDOR_IE_OUI2;
  ie[5] = AVB_VENDOR_IE_OUI_TYPE;
  /* ie[6..] is FollowUpInformation — zero-filled stub. */

  esp_err_t r = esp_hosted_send_custom_data(PTP_RPC_MSG_SET_VENDOR_IE_REQ, buf,
                                            sizeof(buf));
  if (r == ESP_OK) {
    ESP_LOGI(TAG,
             "Beacon Vendor IE publish dispatched: OUI %02x:%02x:%02x "
             "type %d (FollowUpInformation stub, %d bytes payload). "
             "ACK comes back on PTP_RPC_MSG_SET_VENDOR_IE_ACK.",
             AVB_VENDOR_IE_OUI0, AVB_VENDOR_IE_OUI1, AVB_VENDOR_IE_OUI2,
             AVB_VENDOR_IE_OUI_TYPE, AVB_FOLLOWUP_INFO_LEN);
  } else {
    ESP_LOGE(TAG, "esp_hosted_send_custom_data(SET_VENDOR_IE_REQ): %s",
             esp_err_to_name(r));
  }
  return r;
}

/* Fires when the coprocessor's SoftAP comes up. At this moment
 * esp_wifi_set_vendor_ie() will actually take effect; firing earlier
 * would race against the AP-mode init on the coprocessor. */
static void on_wifi_ap_start(void *arg, esp_event_base_t base, int32_t id,
                             void *event_data) {
  (void)arg;
  (void)base;
  (void)id;
  (void)event_data;
  (void)publish_followup_beacon_ie_stub();
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
 *      event fires our handler dispatches the SET_VENDOR_IE_REQ.
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
