/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t source_port[10];
    int8_t log_interval;
} ptp_capable_message_t;

typedef struct {
    uint8_t source_port[10];
    int8_t log_link_delay, log_sync, log_announce;
    uint8_t flags;
} ptp_interval_message_t;

enum { PTP_SIGNALING_MALFORMED = -1, PTP_SIGNALING_IGNORED = 0,
       PTP_SIGNALING_CAPABLE = 1, PTP_SIGNALING_CAPABLE_INTERVAL = 2,
       PTP_SIGNALING_MESSAGE_INTERVAL = 3 };

/* Validate the whole chain before exposing any recognized TLV. */
static inline int ptp_signaling_find_tlv(const uint8_t *message, size_t received,
    uint8_t domain, const uint8_t local_port[10], uint8_t subtype,
    const uint8_t **result)
{
    if (!message || !local_port || !result || received < 44)
        return PTP_SIGNALING_MALFORMED;
    size_t length = ((size_t)message[2] << 8) | message[3];
    if (length < 44 || length > received) return PTP_SIGNALING_MALFORMED;
    if (message[0] != 0x1c || (message[1] & 15) != 2 || message[4] != domain ||
        message[5] != 0)
        return PTP_SIGNALING_IGNORED;
    static const uint8_t wildcard[10] = {
        255,255,255,255,255,255,255,255,255,255
    };
    if (memcmp(message + 34, wildcard, 10) && memcmp(message + 34, local_port, 10))
        return PTP_SIGNALING_IGNORED;
    if (!memcmp(message + 20, local_port, 8)) return PTP_SIGNALING_IGNORED;
    const uint8_t *found = NULL;
    unsigned known_seen = 0;
    for (size_t offset = 44; offset < length;) {
        if (length - offset < 4) return PTP_SIGNALING_MALFORMED;
        const uint8_t *tlv = message + offset;
        size_t body = ((size_t)tlv[2] << 8) | tlv[3];
        if (body > length - offset - 4 || (body & 1)) return PTP_SIGNALING_MALFORMED;
        if (body >= 6 && tlv[4] == 0 && tlv[5] == 0x80 && tlv[6] == 0xc2 &&
            tlv[7] == 0 && tlv[8] == 0) {
            unsigned kind = tlv[9];
            /* Message interval requests propagate (0x0003); the gPTP-capable
             * TLV and its interval request do not (0x8000). */
            unsigned expected_type = kind == 2 ? 0x0003 : 0x8000;
            unsigned type = ((unsigned)tlv[0] << 8) | tlv[1];
            if ((kind == 2 || kind == 4 || kind == 5) && type == expected_type) {
                unsigned bit = 1U << kind;
                size_t expected_body = kind == 5 ? 10 : 12;
                if (body != expected_body || (known_seen & bit))
                    return PTP_SIGNALING_MALFORMED;
                known_seen |= bit;
                if (kind == subtype) found = tlv;
            }
        }
        offset += body + 4;
    }
    if (!found) return PTP_SIGNALING_IGNORED;
    *result = found;
    return 1;
}

static inline int ptp_signaling_read_capable_tlv(const uint8_t *message, size_t received,
    uint8_t domain, const uint8_t local_port[10], ptp_capable_message_t *result,
    uint8_t subtype, int decoded_kind)
{
    if (!result) return PTP_SIGNALING_MALFORMED;
    const uint8_t *tlv;
    int decoded = ptp_signaling_find_tlv(message, received, domain, local_port, subtype, &tlv);
    if (decoded <= 0) return decoded;
    ptp_capable_message_t next = {.log_interval = (int8_t)tlv[10]};
    memcpy(next.source_port, message + 20, 10);
    *result = next;
    return decoded_kind;
}

/* Parsing does not apply interval policy or establish media qualification. */
static inline int ptp_signaling_read_message_interval(const uint8_t *message, size_t received,
    uint8_t domain, const uint8_t local_port[10], ptp_interval_message_t *result)
{
    if (!result) return PTP_SIGNALING_MALFORMED;
    const uint8_t *tlv;
    int decoded = ptp_signaling_find_tlv(message, received, domain, local_port, 2, &tlv);
    if (decoded <= 0) return decoded;
    ptp_interval_message_t next = {
        .log_link_delay = (int8_t)tlv[10], .log_sync = (int8_t)tlv[11],
        .log_announce = (int8_t)tlv[12], .flags = tlv[13]
    };
    memcpy(next.source_port, message + 20, 10);
    *result = next;
    return PTP_SIGNALING_MESSAGE_INTERVAL;
}

static inline int ptp_signaling_read_capable(const uint8_t *message, size_t received,
    uint8_t domain, const uint8_t local_port[10], ptp_capable_message_t *result)
{
    return ptp_signaling_read_capable_tlv(message, received, domain, local_port,
        result, 4, PTP_SIGNALING_CAPABLE);
}

static inline int ptp_signaling_read_capable_interval(const uint8_t *message, size_t received,
    uint8_t domain, const uint8_t local_port[10], ptp_capable_message_t *result)
{
    return ptp_signaling_read_capable_tlv(message, received, domain, local_port,
        result, 5, PTP_SIGNALING_CAPABLE_INTERVAL);
}

#define PTP_CAPABLE_MESSAGE_LENGTH 60

/* Capability indication, separate from media qualification and interval requests. */
static inline size_t ptp_signaling_write_capable(uint8_t *message, size_t capacity,
    const uint8_t source_port[10], uint8_t domain, uint16_t sequence, int8_t log_interval)
{
    if (!message || !source_port || capacity < PTP_CAPABLE_MESSAGE_LENGTH ||
        log_interval < -24 || log_interval > 24) return 0;
    memset(message, 0, PTP_CAPABLE_MESSAGE_LENGTH);
    message[0] = 0x1c;
    message[1] = 0x12;
    message[3] = PTP_CAPABLE_MESSAGE_LENGTH;
    message[4] = domain;
    memcpy(message + 20, source_port, 10);
    message[30] = sequence >> 8;
    message[31] = sequence;
    message[33] = 0x7f;
    memset(message + 34, 0xff, 10);
    const uint8_t tlv[16] = {0x80,0,0,12,0,0x80,0xc2,0,0,4,0,0,0,0,0,0};
    memcpy(message + 44, tlv, sizeof(tlv));
    message[54] = (uint8_t)log_interval;
    return PTP_CAPABLE_MESSAGE_LENGTH;
}

#define PTP_CAPABLE_INTERVAL_MESSAGE_LENGTH 58

/* Separate request TLV, 126 resets, 127 stops, -128 leaves the interval unchanged. */
static inline size_t ptp_signaling_write_capable_interval(uint8_t *message, size_t capacity,
    const uint8_t source_port[10], uint8_t domain, uint16_t sequence, int8_t log_interval)
{
    if (!message || !source_port || capacity < PTP_CAPABLE_INTERVAL_MESSAGE_LENGTH ||
        !((log_interval >= -24 && log_interval <= 24) ||
          log_interval == -128 || log_interval == 126 || log_interval == 127)) return 0;
    uint8_t encoded[PTP_CAPABLE_MESSAGE_LENGTH];
    if (!ptp_signaling_write_capable(encoded, sizeof(encoded), source_port, domain, sequence, 0))
        return 0;
    encoded[3] = PTP_CAPABLE_INTERVAL_MESSAGE_LENGTH;
    encoded[47] = 10;
    encoded[53] = 5;
    encoded[54] = (uint8_t)log_interval;
    memcpy(message, encoded, PTP_CAPABLE_INTERVAL_MESSAGE_LENGTH);
    return PTP_CAPABLE_INTERVAL_MESSAGE_LENGTH;
}
