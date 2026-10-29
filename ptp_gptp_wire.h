#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Normalize locally transmitted gPTP fields, preserving timing metadata.
 * minor_version is the minorVersionPTP to send (0 for 802.1AS-2011, 1
 * for 2020). */
static inline void ptp_gptp_wire_normalize(uint8_t *wire, size_t length, bool gptp,
                                           uint8_t minor_version)
{
    if (!gptp || !wire || length < 34) return;
    wire[1] = (uint8_t)(minor_version << 4) | 0x02;
    wire[5] = 0;
    memset(wire + 16, 0, 4);
    wire[32] = 0;
    unsigned type = wire[0] & 15;
    if (type == 12) wire[33] = 0x7f;
    if (type == 11 && length >= 64) {
        memset(wire + 34, 0, 10);
        wire[46] = 0;
    } else if (type == 2 && length >= 54) {
        memset(wire + 34, 0, 20);
    } else if (type == 0 && (wire[6] & 2) && length >= 44) {
        memset(wire + 34, 0, 10);
    }
}
