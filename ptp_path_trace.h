/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PTP_ANNOUNCE_BODY_LENGTH 64
#define PTP_PATH_TRACE_MAX_CLOCKS 16
#define PTP_ANNOUNCE_MAX_LENGTH \
  (PTP_ANNOUNCE_BODY_LENGTH + 4 + PTP_PATH_TRACE_MAX_CLOCKS * 8)

typedef struct {
  unsigned count;
  uint8_t identities[PTP_PATH_TRACE_MAX_CLOCKS][8];
} ptp_path_trace_t;

/* Missing optional TLV produces an empty path; malformed or looped paths fail. */
static inline bool ptp_path_trace_parse(const uint8_t *message, size_t received,
                                        const uint8_t local_identity[8],
                                        ptp_path_trace_t *path) {
  if (!message || !local_identity || !path || received < PTP_ANNOUNCE_BODY_LENGTH)
    return false;
  size_t length = ((size_t)message[2] << 8) | message[3];
  if (length < PTP_ANNOUNCE_BODY_LENGTH || length > received)
    return false;
  ptp_path_trace_t parsed = {0};
  bool found = false;
  size_t offset = PTP_ANNOUNCE_BODY_LENGTH;
  while (offset < length) {
    if (length - offset < 4)
      return false;
    unsigned type = ((unsigned)message[offset] << 8) | message[offset + 1];
    size_t bytes = ((size_t)message[offset + 2] << 8) | message[offset + 3];
    offset += 4;
    if (bytes > length - offset)
      return false;
    if (type == 8) {
      if (found || !bytes || bytes % 8 || bytes / 8 > PTP_PATH_TRACE_MAX_CLOCKS)
        return false;
      found = true;
      parsed.count = bytes / 8;
      for (unsigned index = 0; index < parsed.count; ++index) {
        const uint8_t *identity = message + offset + index * 8;
        if (!memcmp(identity, local_identity, 8))
          return false;
        memcpy(parsed.identities[index], identity, 8);
      }
    }
    offset += bytes;
  }
  *path = parsed;
  return true;
}

/* Append this clock exactly once. Refuse overflow instead of truncating hops. */
static inline size_t ptp_path_trace_write(uint8_t *output, size_t capacity,
                                         const ptp_path_trace_t *upstream,
                                         const uint8_t local_identity[8]) {
  if (!output || !upstream || !local_identity ||
      upstream->count >= PTP_PATH_TRACE_MAX_CLOCKS)
    return 0;
  size_t bytes = (upstream->count + 1) * 8;
  if (capacity < bytes + 4)
    return 0;
  for (unsigned index = 0; index < upstream->count; ++index) {
    if (!memcmp(upstream->identities[index], local_identity, 8))
      return 0;
  }
  output[0] = 0;
  output[1] = 8;
  output[2] = bytes >> 8;
  output[3] = bytes;
  memcpy(output + 4, upstream->identities, upstream->count * 8);
  memcpy(output + 4 + upstream->count * 8, local_identity, 8);
  return bytes + 4;
}
