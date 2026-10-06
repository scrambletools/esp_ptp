#pragma once
#include "ptp_capable_receive.h"
#include "ptp_capable_schedule.h"
#include "ptp_wifi_association.h"
#include "ptp_sync_interval.h"
#include "ptp_announce_schedule.h"

#define PTP_WIFI_PEERS_MAX PTP_WIFI_ASSOCIATIONS_MAX

typedef struct {
    bool associated, bound, sync_stopped;
    uint8_t mac[6], port_identity[10];
    uint32_t association, sync_revision;
    ptp_capable_receive_t capable;
    ptp_capable_schedule_t transmit;
    ptp_announce_schedule_t announce;
} ptp_wifi_peer_t;

typedef struct {
    uint32_t generation, radio_boot, radio_generation, reconciled_generation;
    ptp_wifi_peer_t entries[PTP_WIFI_PEERS_MAX];
} ptp_wifi_peers_t;

/* Hosted AP identities are publishable only after authoritative reconciliation. */
static inline bool ptp_wifi_peers_publishable(const ptp_wifi_peers_t *peers)
{
#if CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR
    return peers && peers->radio_boot && peers->radio_generation &&
           peers->reconciled_generation == peers->generation;
#else
    return peers != NULL;
#endif
}

static inline uint32_t ptp_wifi_peers_advance(ptp_wifi_peers_t *peers)
{
    if (!++peers->generation) ++peers->generation;
    return peers->generation;
}

static inline ptp_wifi_peer_t *ptp_wifi_peers_find(ptp_wifi_peers_t *peers,
                                                 const uint8_t mac[6])
{
    if (!mac) return NULL;
    for (unsigned index = 0; index < PTP_WIFI_PEERS_MAX; ++index)
        if (peers->entries[index].associated && !memcmp(peers->entries[index].mac, mac, 6))
            return &peers->entries[index];
    return NULL;
}

static inline void ptp_wifi_peers_clear(ptp_wifi_peers_t *peers)
{
    ptp_wifi_peers_advance(peers);
    memset(peers->entries, 0, sizeof(peers->entries));
}

/* An association event always starts a new lifetime, even for the same MAC. */
static inline ptp_wifi_peer_t *ptp_wifi_peers_join(ptp_wifi_peers_t *peers,
                                                 const uint8_t mac[6])
{
    static const uint8_t zero[6];
    if (!mac || (mac[0] & 1) || !memcmp(mac, zero, 6)) return NULL;
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    if (!peer) {
        for (unsigned index = 0; index < PTP_WIFI_PEERS_MAX; ++index)
            if (!peers->entries[index].associated) { peer = &peers->entries[index]; break; }
    }
    if (!peer) return NULL;
    memset(peer, 0, sizeof(*peer));
    peer->associated = true;
    memcpy(peer->mac, mac, 6);
    peer->association = ptp_wifi_peers_advance(peers);
    return peer;
}

static inline void ptp_wifi_peers_leave(ptp_wifi_peers_t *peers,
                                       const uint8_t mac[6])
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    if (!peer) return;
    memset(peer, 0, sizeof(*peer));
    ptp_wifi_peers_advance(peers);
}

/* Signaling identifies the PTP neighbor only after MAC association validation. */
static inline bool ptp_wifi_peers_capable(ptp_wifi_peers_t *peers,
    const uint8_t mac[6], uint32_t association,
    const ptp_capable_message_t *message, int64_t now_us)
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    if (!peer || peer->association != association ||
        (!message->source_port[8] && !message->source_port[9])) return false;
    if (peer->bound && memcmp(peer->port_identity, message->source_port, 10)) {
        peer->capable.valid = false;
        ptp_announce_reset(&peer->announce);
        ptp_sync_interval_reset(&peer->sync_stopped, &peer->sync_revision);
        uint32_t serial = peer->transmit.serial;
        memset(&peer->transmit, 0, sizeof(peer->transmit));
        peer->transmit.serial = serial;
    }
    if (!ptp_capable_receive(&peer->capable, message, message->source_port,
                             association, now_us)) return false;
    memcpy(peer->port_identity, message->source_port, 10);
    peer->bound = true;
    return true;
}

/* Interval changes require the identity already bound to this association. */
static inline bool ptp_wifi_peers_interval(ptp_wifi_peers_t *peers,
    const uint8_t mac[6], uint32_t association,
    const ptp_capable_message_t *message)
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    return peer && peer->association == association && peer->bound &&
        !memcmp(peer->port_identity, message->source_port, 10) &&
        ptp_capable_schedule_request(&peer->transmit, message->log_interval);
}

/* Supports the default sync rate and stop; optional rates remain unsupported. */
static inline bool ptp_wifi_peers_sync_interval(ptp_wifi_peers_t *peers,
    const uint8_t mac[6], uint32_t association, const ptp_interval_message_t *message)
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    if (!message || !peer || peer->association != association || !peer->bound ||
        memcmp(peer->port_identity, message->source_port, 10)) return false;
    return ptp_sync_interval_request(&peer->sync_stopped, &peer->sync_revision,
                                      message->log_sync);
}

static inline bool ptp_wifi_peers_announce_interval(ptp_wifi_peers_t *peers,
    const uint8_t mac[6], uint32_t association, const ptp_interval_message_t *message,
    int8_t initial)
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    return message && peer && peer->association == association && peer->bound &&
        !memcmp(peer->port_identity, message->source_port, 10) &&
        ptp_announce_request(&peer->announce, initial, message->log_announce);
}

static inline bool ptp_wifi_peers_sent(ptp_wifi_peers_t *peers,
    const uint8_t mac[6], uint32_t association, uint32_t token,
    bool accepted, int64_t now_us)
{
    ptp_wifi_peer_t *peer = ptp_wifi_peers_find(peers, mac);
    return peer && peer->association == association &&
        ptp_capable_schedule_finish(&peer->transmit, token, accepted, now_us);
}

/* Caller holds the peer lock. Copy by slot so removals never renumber survivors. */
static inline bool ptp_wifi_peers_snapshot(const ptp_wifi_peers_t *peers,
    unsigned physical_port, unsigned physical_count,
    ptp_wifi_association_snapshot_t *snapshot)
{
    if (!peers || !snapshot ||
        !ptp_wifi_association_port(physical_port, physical_count, 0)) return false;
    ptp_wifi_association_snapshot_t next = {.generation = peers->generation};
    if (peers->reconciled_generation == peers->generation) {
        next.radio_boot = peers->radio_boot;
        next.radio_generation = peers->radio_generation;
    }
    for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
        const ptp_wifi_peer_t *peer = &peers->entries[slot];
        if (!peer->associated) continue;
        if (!peer->association) return false;
        ptp_wifi_association_entry_t *entry = &next.entries[next.count++];
        memcpy(entry->mac, peer->mac, sizeof(entry->mac));
        entry->port_number = ptp_wifi_association_port(physical_port, physical_count, slot);
        entry->association = peer->association;
        entry->sync_stopped = peer->sync_stopped;
        entry->sync_revision = peer->sync_revision;
        entry->announce_revision = peer->announce.revision;
    }
    *snapshot = next;
    return true;
}

/* Reset media state at a new radio lifetime, preserving surviving logical slots. */
static inline bool ptp_wifi_peers_reconcile(ptp_wifi_peers_t *peers,
    uint32_t radio_boot, uint32_t radio_generation, const uint8_t macs[][6], unsigned count)
{
    if (!peers || !radio_boot || !radio_generation || count > PTP_WIFI_PEERS_MAX ||
        (!macs && count)) return false;
    const uint8_t zero[6] = {0};
    for (unsigned index = 0; index < count; ++index) {
        if ((macs[index][0] & 1) || !memcmp(macs[index], zero, 6)) return false;
        for (unsigned previous = 0; previous < index; ++previous)
            if (!memcmp(macs[index], macs[previous], 6)) return false;
    }
    if (peers->radio_boot == radio_boot) {
        int32_t difference = (int32_t)(radio_generation - peers->radio_generation);
        if (difference < 0) return false;
        if (!difference) {
            if (peers->reconciled_generation != peers->generation) return false;
            unsigned associated = 0;
            for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
                const ptp_wifi_peer_t *peer = &peers->entries[slot];
                if (!peer->associated) continue;
                ++associated;
                bool found = false;
                for (unsigned index = 0; index < count; ++index)
                    if (!memcmp(peer->mac, macs[index], 6)) { found = true; break; }
                if (!found) return false;
            }
            return associated == count;
        }
    }
    for (unsigned slot = 0; slot < PTP_WIFI_PEERS_MAX; ++slot) {
        ptp_wifi_peer_t *peer = &peers->entries[slot];
        if (!peer->associated) continue;
        bool found = false;
        for (unsigned index = 0; index < count; ++index)
            if (!memcmp(peer->mac, macs[index], 6)) { found = true; break; }
        if (!found) memset(peer, 0, sizeof(*peer));
    }
    ptp_wifi_peers_advance(peers);
    for (unsigned index = 0; index < count; ++index)
        (void)ptp_wifi_peers_join(peers, macs[index]);
    peers->radio_boot = radio_boot;
    peers->radio_generation = radio_generation;
    peers->reconciled_generation = peers->generation;
    return true;
}
