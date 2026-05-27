/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 * SPDX-FileContributor: 2024 Espressif Systems (Shanghai) CO LTD
 */

#ifndef _ESP_PTP_PTP_H_
#define _ESP_PTP_PTP_H_

#include <stdint.h>
#include <time.h>

/* Time-critical messages (id < 8) go to port 319, others to 320. */

#define PTP_UDP_PORT_EVENT 319
#define PTP_UDP_PORT_INFO 320

/* Multicast address to send to: 224.0.1.129 */

#define PTP_MULTICAST_ADDR ((in_addr_t)0xE0000181)

/* Multicast MAC addresses for PTP */

/* gPTP uses LLDP nearest-bridge; non-gPTP uses the standard PTP group. */
#define LLDP_MULTICAST_ADDR (uint8_t[6]){0x01, 0x80, 0xC2, 0x00, 0x00, 0x0e}
#define PTP4L_MULTICAST_ADDR (uint8_t[6]){0x01, 0x1B, 0x19, 0x00, 0x00, 0x00}

/* Message types */

#define PTP_MSGTYPE_MASK 0x0F
#define PTP_MSGTYPE_SYNC 0x00
#define PTP_MSGTYPE_FOLLOW_UP 0x08
#define PTP_MSGTYPE_ANNOUNCE 0x0b
#define PTP_MSGTYPE_DELAY_REQ 0x01
#define PTP_MSGTYPE_DELAY_RESP 0x09
#define PTP_MSGTYPE_PDELAY_REQ 0x02
#define PTP_MSGTYPE_PDELAY_RESP 0x03
#define PTP_MSGTYPE_PDELAY_RESP_FOLLOW_UP 0x0a

/* Message flags */

#define PTP_FLAGS0_TWOSTEP (1 << 1)
#define PTP_FLAGS1_PTP_TIMESCALE (1 << 3)
#define PTP_MSGTYPE_SDOID_GPTP (1 << 4)

/* Wire format per IEEE 1588-2019 / IEEE 802.1AS-2020. Multi-byte
 * fields are big-endian. */

/* Path trace TLV for gPTP follow up messages */

struct ptp_pathtrace_tlv_s {
  uint8_t type[2];
  uint8_t length[2];
  uint8_t pathsequence[8]; // this can have more but gPTP endpoints will ignore
};

/* Information TLV for gPTP announce messages */

struct ptp_info_tlv_s {
  uint8_t type[2];
  uint8_t length[2];
  uint8_t orgidentity[3];
  uint8_t orgsubtype[3];
  uint8_t cumulativescaledrateoffset[4];
  uint8_t gmtimebaseindicator[2];
  uint8_t lastgmphasechange[12];
  uint8_t scaledlastgmfreqchange[4];
};

/* Common header for all message types */

struct ptp_header_s {
  uint8_t messagetype;
  uint8_t version;
  uint8_t messagelength[2];
  uint8_t domain;
  uint8_t reserved1;
  uint8_t flags[2];
  uint8_t correction[8];
  uint8_t reserved2[4];
  uint8_t sourceidentity[8];
  uint8_t sourceportindex[2];
  uint8_t sequenceid[2];
  uint8_t controlfield;
  uint8_t logmessageinterval;
};

/* Announce a timetransmitter clock */

struct ptp_announce_s {
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
  uint8_t utcoffset[2];
  uint8_t reserved;
  uint8_t btc_priority1;
  uint8_t btc_quality[4];
  uint8_t btc_priority2;
  uint8_t btc_identity[8];
  uint8_t stepsremoved[2];
  uint8_t timesource;
  struct ptp_pathtrace_tlv_s pathtracetlv; // gPTP required
};

/* Sync: transmit timestamp from timetransmitter clock */

struct ptp_sync_s {
  struct ptp_header_s header;
  uint8_t origintimestamp[10]; // in gPTP profile, this will be ignored
};

/* FollowUp: actual timestamp of when sync message was sent */

struct ptp_follow_up_s {
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
  uint8_t informationtlv[32]; // gPTP required
};

/* DelayReq: request delay measurement (path delay) */

struct ptp_delay_req_s {
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
};

/* DelayResp: response to DelayReq (path delay or peer delay)*/

struct ptp_delay_resp_s {
  struct ptp_header_s header;
  uint8_t receivetimestamp[10];
  uint8_t reqidentity[8];
  uint8_t reqportindex[2];
};

/* DelayResp: follow up to DelayResp (gPTP only)*/

struct ptp_delay_resp_follow_up_s {
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
  uint8_t reqidentity[8];
  uint8_t reqportindex[2];
};

/* PeerDelayReq: request peer delay measurement */

struct ptp_pdelay_req_s {
  struct ptp_header_s header;
  uint8_t reserved[20];
};

/* AVB Lite Endpoint Declaration TLV — per profiles/avb_lite.md §2.1.
 * Carried in Pdelay_Req, Pdelay_Resp, and Pdelay_Resp_Follow_Up to let
 * AVB Lite endpoints detect each other across non-AVB-aware bridges. */

#define PTP_TLV_TYPE_ORGANIZATION_EXTENSION 0x0003

#define PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES {0x8C, 0x1F, 0x64}
#define PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES {0x36, 0xC0, 0x01}
#define PTP_ENDPOINT_DECL_TLV_DATA 0x01

struct ptp_endpoint_decl_tlv_s {
  uint8_t type[2];
  uint8_t length[2];
  uint8_t orgidentity[3];
  uint8_t orgsubtype[3];
  uint8_t data;
};

/* Internal API — visible only within the esp_ptp component, not part
 * of the public esp_ptp.h surface. */

/* Defined in ptp.c. Pushed by the per-medium transport modules into
 * the daemon. Kept here (not in include/esp_ptp.h) so applications
 * can't reach into the protocol guts — they only call
 * ptpd_start_port() and let esp_ptp own the wire. */
int ptpd_inject_peer_delay(int port_index, int64_t peer_delay_ns);
int ptpd_inject_sync(int port_index, const uint8_t *follow_up_info, size_t len);
int ptpd_inject_sync_pair(int port_index, int64_t remote_ns, int64_t local_ns);

/* ptp_inject_received_frame() is now part of the public esp_ptp.h
 * surface — esp_avb's Wi-Fi RX dispatcher calls it directly for
 * 0x88f7 frames (the IDF Wi-Fi netif has no L2TAP backend, so the
 * fan-out can't be delegated to L2TAP the way the Ethernet path
 * does). See include/esp_ptp.h. */

/* Defined in ptp_wifi.c. Spun up by ptp_port_init_wifi_ftm when
 * wifi_mode == sta. Owns the §12.7 beacon-IE parser, the FTM
 * initiator burst loop, and the FTM_REPORT handler that feeds
 * inject_peer_delay + inject_sync_pair. Idempotent. */
int ptp_wifi_sta_start(int port_index);

/* Defined in ptp_beacon_ie.c (only compiled when
 * CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR=y). Registers the §12.7
 * FollowUpInformation beacon-IE publisher as ptpd's sync_egress_cb
 * for the given AP-mode wifi_ftm port. Must be called AFTER ptpd has
 * been started (s_state != NULL), so the registration won't fail with
 * -ESRCH. Called from ptp_port_init_wifi_ftm in the AP branch.
 * Returns 0 on success, negative errno otherwise. The non-AP build
 * variant is a no-op (weak symbol below). */
int ptp_beacon_ie_attach(int port_index);

/* Defined in ptp_wifi.c. Emits one unicast 802.1AS Announce per
 * currently-associated STA on a wifi_ftm/AP port. Per IEEE 802.1AS-
 * 2020 §12.2, media-independent messages (Announce, Signaling) are
 * sent as unicast to each STA on Wi-Fi rather than the group address.
 * ptp_msg is the already-marshalled Announce body (header + body +
 * path-trace TLV); src_mac is the AP port's MAC; the helper wraps
 * the body in Ethernet (dst = STA MAC, src = src_mac, ethertype
 * 0x88f7) and pushes it via esp_wifi_internal_tx(WIFI_IF_AP, ...). */
int ptp_wifi_ap_send_announce(int port_index, const uint8_t src_mac[6],
                              void *ptp_msg, uint16_t ptp_msg_len);

/* Defined in ptp.c. Register a callback fired whenever the daemon
 * would emit a gPTP Sync/Follow_Up on port_index. The callback
 * carries the marshalled FollowUpInformation TLV to the wire (e.g.
 * beacon Vendor IE update). One callback per port; NULL clears.
 *
 * Kept here (not in include/esp_ptp.h) so applications can't reach
 * into the protocol guts — the only consumer is ptp_beacon_ie.c
 * (autoregistered via constructor), which carries the bytes to the
 * coprocessor via esp_hosted RPC. */
typedef void (*ptpd_sync_egress_cb_t)(int port_index,
                                      const uint8_t *follow_up_info, size_t len,
                                      void *ctx);

int ptpd_register_sync_egress_cb(int port_index, ptpd_sync_egress_cb_t cb,
                                 void *ctx);

/* ---- Software clock backend (ptp_clock_sw.c) ----
 *
 * Used when CLOCK_PTP_SYSTEM is not pluggable (e.g. ESP32-C6).
 * Disciplines (offset_ns, rate_ppb) over esp_timer_get_time().
 * ptpd_now() routes through this once ptp_clock_sw_init() runs; on
 * EMAC 1588 platforms it is unused. */

/* Initialize and select the software clock as the backend for
 * ptpd_now(). Idempotent. After this call ptpd_now() returns the
 * disciplined software time instead of clock_gettime(CLOCK_PTP_SYSTEM).
 * initial_ts may be NULL, in which case the clock starts at 0. */
int ptp_clock_sw_init(const struct timespec *initial_ts);

/* Read the current software time. */
int ptp_clock_sw_now(struct timespec *ts);

/* Step the software time to ts. Discontinuous — re-anchors immediately. */
int ptp_clock_sw_settime(const struct timespec *ts);

/* Slew the software time by delta_ns. Continuous — preserves rate. */
int ptp_clock_sw_adjtime_offset(int64_t delta_ns);

/* Set the rate offset, in parts-per-billion relative to the local
 * esp_timer. Positive ppb means the software clock runs faster than
 * the local oscillator. Range +/- 1e6 ppb (+/- 1000 ppm) is
 * comfortable; anything beyond ±1e8 is rejected. Re-anchors so the
 * change does not introduce a discontinuity. */
int ptp_clock_sw_adjtime_rate(int32_t rate_ppb);

#endif /* __APPS_ESP_PTP_PTPV2_H */
