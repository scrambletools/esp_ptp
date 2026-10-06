#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Normalize locally transmitted gPTP fields, preserving timing metadata. */
static inline void ptp_gptp_wire_normalize(uint8_t *wire, size_t length, bool gptp)
{
    if (!gptp || !wire || length < 34) return;
    wire[1] = 0x12;
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
