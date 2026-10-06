#pragma once
#include <stdbool.h>
#include <stdint.h>

static inline void ptp_sync_interval_reset(bool *stopped, uint32_t *revision)
{
    *stopped = false;
    if (!++*revision) ++*revision;
}

/* Default synchronization interval and stop are supported; other rates are optional. */
static inline bool ptp_sync_interval_request(bool *stopped, uint32_t *revision,
                                             int8_t requested)
{
    bool next = *stopped;
    switch (requested) {
    case -128: return true;
    case -3:
    case 126: next = false; break;
    case 127: next = true; break;
    default: return false;
    }
    if (next != *stopped) {
        *stopped = next;
        if (!++*revision) ++*revision;
    }
    return true;
}
