#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ptp_signaling.h"
#include "ptp_capable_schedule.h"
#include "ptp_sync_interval.h"

typedef struct {
    uint32_t association, sync_revision;
    uint8_t bssid[6], port_identity[10];
    bool associated, bound, sync_stopped;
    ptp_capable_schedule_t transmit;
} ptp_wifi_neighbor_t;

static inline void ptp_wifi_neighbor_associate(ptp_wifi_neighbor_t *neighbor,
                                              const uint8_t bssid[6])
{
    uint32_t association = neighbor->association + 1;
    memset(neighbor, 0, sizeof(*neighbor));
    neighbor->association = association ? association : 1;
    if (bssid) {
        memcpy(neighbor->bssid, bssid, 6);
        neighbor->associated = true;
    }
}

static inline bool ptp_wifi_neighbor_accepts(const ptp_wifi_neighbor_t *neighbor,
                                            const uint8_t source_mac[6])
{
    return neighbor->associated && source_mac &&
           !memcmp(neighbor->bssid, source_mac, 6);
}

/* Learn from a valid Announce, never from a capability indication. */
static inline bool ptp_wifi_neighbor_bind(ptp_wifi_neighbor_t *neighbor,
    uint32_t association, const uint8_t source_mac[6], const uint8_t identity[10])
{
    if (association != neighbor->association ||
        !ptp_wifi_neighbor_accepts(neighbor, source_mac)) return false;
    if (neighbor->bound && memcmp(neighbor->port_identity, identity, 10)) {
        ptp_sync_interval_reset(&neighbor->sync_stopped, &neighbor->sync_revision);
        uint32_t serial = neighbor->transmit.serial;
        memset(&neighbor->transmit, 0, sizeof(neighbor->transmit));
        neighbor->transmit.serial = serial;
    }
    memcpy(neighbor->port_identity, identity, 10);
    neighbor->bound = true;
    return true;
}

static inline bool ptp_wifi_neighbor_interval(ptp_wifi_neighbor_t *neighbor,
    const uint8_t source_mac[6], uint32_t association,
    const ptp_capable_message_t *message)
{
    return neighbor->bound && association == neighbor->association &&
        ptp_wifi_neighbor_accepts(neighbor, source_mac) &&
        !memcmp(neighbor->port_identity, message->source_port, 10) &&
        ptp_capable_schedule_request(&neighbor->transmit, message->log_interval);
}

static inline bool ptp_wifi_neighbor_sync_interval(ptp_wifi_neighbor_t *neighbor,
    const uint8_t source_mac[6], uint32_t association, const ptp_interval_message_t *message)
{
    return message && neighbor->bound && association == neighbor->association &&
        ptp_wifi_neighbor_accepts(neighbor, source_mac) &&
        !memcmp(neighbor->port_identity, message->source_port, 10) &&
        ptp_sync_interval_request(&neighbor->sync_stopped, &neighbor->sync_revision,
                                  message->log_sync);
}

static inline bool ptp_wifi_neighbor_sent(ptp_wifi_neighbor_t *neighbor,
    const uint8_t destination[6], uint32_t association, uint32_t token,
    bool accepted, int64_t now_us)
{
    return association == neighbor->association &&
        ptp_wifi_neighbor_accepts(neighbor, destination) &&
        ptp_capable_schedule_finish(&neighbor->transmit, token, accepted, now_us);
}
