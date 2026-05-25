/*
 * Wi-Fi AP-side PTP transport for ports with medium = wifi_ftm
 * and wifi_mode = ap. Implements IEEE 802.1AS-2020 §12.2 unicast
 * Announce: the bridge re-emits the upstream BTC's Announce as one
 * unicast 802.1AS frame per associated STA so the STA's existing
 * BMCA receives the real GM priority / clockQuality (the §12.7
 * beacon IE only carries Sync timing).
 */

#include "sdkconfig.h"

#include <inttypes.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"

#include "esp_ptp.h"
#include "ptp.h"

/* esp_wifi_internal_tx is not in any public IDF header. Same forward-
 * declaration pattern as in esp_avb/avbnet.c. The wifi_remote layer
 * on a host with a coprocessor radio forwards this call over SDIO so
 * the AP actually emits the frame from the C6 side. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

#define TAG "ptp_wifi_ap"

#define ETH_HDR_LEN 14

/* Build an Ethernet frame with PTP ethertype carrying ptp_msg, then
 * push it as one unicast TX to dst_mac via WIFI_IF_AP. Caller-owned
 * buffer; we allocate on stack since Announce is small (~88 B). */
static int wifi_ap_send_unicast_ptp(const uint8_t src_mac[6],
                                    const uint8_t dst_mac[6],
                                    void *ptp_msg,
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

int ptp_wifi_ap_send_announce(int port_index,
                              const uint8_t src_mac[6],
                              void *ptp_msg,
                              uint16_t ptp_msg_len) {
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
    int rc = wifi_ap_send_unicast_ptp(src_mac, sta_list.sta[i].mac,
                                      ptp_msg, ptp_msg_len);
    if (rc > 0) sent++;
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
