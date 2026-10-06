#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PTP_TIMING_FOLLOW_UP_SIZE 76

typedef struct {
    uint32_t generation;
    int64_t received_us;
    int64_t reference_ns;
    int64_t local_receive_ns;
    int64_t offset_ns;
    int64_t correction_ns;
    int32_t peer_delay_ns;
    int32_t trim_ppb;
    int32_t drift_ppb;
    uint8_t btc_identity[8];
    uint8_t own_identity[8];
    uint8_t follow_up[PTP_TIMING_FOLLOW_UP_SIZE];
    bool hardware_clock;
    bool valid;
} ptp_timing_snapshot_t;

/* Task-context read, expires after 500 ms. Valid means a recent accepted
 * upstream observation with its information TLV, not a phase-lock claim. */
bool ptpd_timing_snapshot(ptp_timing_snapshot_t *snapshot);

/* Retires queued source information even when no fresh timing is available. */
uint32_t ptpd_timing_generation(void);

/* Selected timing for downstream origination, including a local hardware root.
 * A local source retains the running oscillator, without claiming traceability. */
bool ptpd_time_source_snapshot(ptp_timing_snapshot_t *snapshot);

static inline bool ptp_timing_payload_valid(const uint8_t *message, size_t size)
{
    if (!message || size < PTP_TIMING_FOLLOW_UP_SIZE) return false;
    unsigned length = ((unsigned)message[2] << 8) | message[3];
    if ((message[0] & 15) != 8 || (message[1] & 15) != 2 ||
        length < PTP_TIMING_FOLLOW_UP_SIZE || length > size) return false;
    const uint8_t *tlv = message + 44;
    return tlv[0] == 0 && tlv[1] == 3 && tlv[2] == 0 && tlv[3] == 28 &&
        tlv[4] == 0 && tlv[5] == 0x80 && tlv[6] == 0xc2 &&
        tlv[7] == 0 && tlv[8] == 0 && tlv[9] == 1;
}
