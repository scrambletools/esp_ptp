/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2020-2024 The Apache Software Foundation
 * SPDX-FileContributor: 2024 Espressif Systems (Shanghai) CO LTD
 */

#ifndef __APPS_NETUTILS_PTPD_PTPV2_H
#define __APPS_NETUTILS_PTPD_PTPV2_H

#include <stdint.h>

/* Time-critical messages (id < 8) go to port 319, others to 320. */

#define PTP_UDP_PORT_EVENT 319
#define PTP_UDP_PORT_INFO 320

/* Multicast address to send to: 224.0.1.129 */

#define PTP_MULTICAST_ADDR ((in_addr_t)0xE0000181)

/* Multicast MAC addresses for PTP */

/* gPTP uses LLDP nearest-bridge; non-gPTP uses the standard PTP group. */
#define LLDP_MULTICAST_ADDR                                                    \
  (uint8_t[6]){0x01, 0x80, 0xC2, 0x00, 0x00, 0x0e}
#define PTP4L_MULTICAST_ADDR                                                   \
  (uint8_t[6]){0x01, 0x1B, 0x19, 0x00, 0x00, 0x00}

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

struct ptp_pathtrace_tlv_s
{
  uint8_t type[2];
  uint8_t length[2];
  uint8_t pathsequence[8]; // this can have more but gPTP endpoints will ignore
};

/* Information TLV for gPTP announce messages */

struct ptp_info_tlv_s
{
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

struct ptp_header_s
{
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

struct ptp_announce_s
{
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

struct ptp_sync_s
{
  struct ptp_header_s header;
  uint8_t origintimestamp[10]; // in gPTP profile, this will be ignored
};

/* FollowUp: actual timestamp of when sync message was sent */

struct ptp_follow_up_s
{
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
  uint8_t informationtlv[32]; // gPTP required
};

/* DelayReq: request delay measurement (path delay) */

struct ptp_delay_req_s
{
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
};

/* DelayResp: response to DelayReq (path delay or peer delay)*/

struct ptp_delay_resp_s
{
  struct ptp_header_s header;
  uint8_t receivetimestamp[10];
  uint8_t reqidentity[8];
  uint8_t reqportindex[2];
};

/* DelayResp: follow up to DelayResp (gPTP only)*/

struct ptp_delay_resp_follow_up_s
{
  struct ptp_header_s header;
  uint8_t origintimestamp[10];
  uint8_t reqidentity[8];
  uint8_t reqportindex[2];
};

/* PeerDelayReq: request peer delay measurement */

struct ptp_pdelay_req_s
{
  struct ptp_header_s header;
  uint8_t reserved[20];
};

/* AVB Lite Endpoint Declaration TLV — per profiles/avb_lite.md §2.1.
 * Carried in Pdelay_Req, Pdelay_Resp, and Pdelay_Resp_Follow_Up to let
 * AVB Lite endpoints detect each other across non-AVB-aware bridges. */

#define PTP_TLV_TYPE_ORGANIZATION_EXTENSION 0x0003

#define PTP_ENDPOINT_DECL_TLV_ORG_ID_BYTES   {0x8C, 0x1F, 0x64}
#define PTP_ENDPOINT_DECL_TLV_SUBTYPE_BYTES {0x36, 0xC0, 0x01}
#define PTP_ENDPOINT_DECL_TLV_DATA  0x01

struct ptp_endpoint_decl_tlv_s
{
  uint8_t type[2];
  uint8_t length[2];
  uint8_t orgidentity[3];
  uint8_t orgsubtype[3];
  uint8_t data;
};

#endif /* __APPS_NETUTILS_PTPD_PTPV2_H */
