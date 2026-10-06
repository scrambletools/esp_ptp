#pragma once
#include <stdbool.h>
#include <stdint.h>

#define PTP_WIFI_ASSOCIATIONS_MAX 16

typedef struct {
    uint8_t mac[6];
    uint16_t port_number;
    uint32_t association;
    bool sync_stopped;
    uint32_t sync_revision, announce_revision;
} ptp_wifi_association_entry_t;

typedef struct {
    uint32_t generation, radio_boot, radio_generation;
    unsigned count;
    ptp_wifi_association_entry_t entries[PTP_WIFI_ASSOCIATIONS_MAX];
} ptp_wifi_association_snapshot_t;

/* Slot zero uses its transport's port; extra slots use disjoint reserved blocks. */
static inline uint16_t ptp_wifi_association_port(unsigned physical_port,
                                                unsigned physical_count,
                                                unsigned slot)
{
    if (!physical_port || physical_port > physical_count ||
        physical_count > UINT16_MAX / PTP_WIFI_ASSOCIATIONS_MAX ||
        slot >= PTP_WIFI_ASSOCIATIONS_MAX) return 0;
    if (!slot) return physical_port;
    return physical_count + (physical_port - 1) * (PTP_WIFI_ASSOCIATIONS_MAX - 1) + slot;
}

/* Task context. Returns a copied current AP registry; no internal pointers escape. */
bool ptpd_wifi_association_snapshot(int port_index,
                                    ptp_wifi_association_snapshot_t *snapshot);

/* Called after crossing the host event loop with an authoritative radio snapshot. */
bool ptpd_wifi_association_reconcile(int port_index, uint32_t radio_boot,
    uint32_t radio_generation, const uint8_t macs[][6], unsigned count);

/* Copied STA timing ownership, independent of the AP registry wire format. */
typedef struct {
    uint32_t association, revision;
    uint8_t bssid[6];
    bool stopped;
} ptp_wifi_sta_timing_t;

bool ptpd_wifi_sta_timing_snapshot(int port_index, ptp_wifi_sta_timing_t *snapshot);

uint32_t ptpd_wifi_announce_begin(int port_index, const uint8_t mac[6],
    uint32_t association, uint32_t revision, uint32_t information_revision,
    int8_t initial, int64_t now_us,
    int8_t *advertised, uint16_t *sequence);
bool ptpd_wifi_announce_finish(int port_index, const uint8_t mac[6],
    uint32_t association, uint32_t token, bool accepted, int64_t now_us);
