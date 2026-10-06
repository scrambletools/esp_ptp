#pragma once
#include <stddef.h>
#include <stdint.h>

/* Return the advertised body length, excluding Ethernet padding. */
static inline size_t ptp_message_bounded_length(const uint8_t *message,
                                               size_t received)
{
    if (!message || received < 34 || (message[1] & 15) != 2) return 0;
    size_t length = ((size_t)message[2] << 8) | message[3];
    size_t minimum = 34;
    switch (message[0] & 15) {
    case 0: case 1: minimum = 44; break;
    case 2: case 3: case 9: case 10: minimum = 54; break;
    case 8: minimum = (message[0] >> 4) == 1 ? 76 : 44; break;
    case 11: minimum = 64; break;
    case 12: minimum = 44; break;
    }
    if (length < minimum || length > received) return 0;
    return length;
}
