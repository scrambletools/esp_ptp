#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Internal transport admission, call only from the daemon's FTM tick hook.
 * A matching observation refreshes reception, not a phase-lock claim. */
bool ptpd_ftm_source_observed(int port_index, const uint8_t source_port[10],
    uint8_t domain, int8_t log_interval, int64_t received_us, uint32_t *generation);

/* Mode A station state for the AVB Wireless status query, filled by the FTM
 * transport's ptp_ftm_status_hook. applied_us is the esp_timer time of the
 * last applied observation, 0 for none; success is a percent over the last
 * 10 s, 0xFF with no measurement attempted. */
typedef struct {
    int64_t applied_us;
    int32_t error_ns, rtt_ns;
    uint8_t success;
    bool locked, holdover, error_valid, rtt_valid;
} ptp_ftm_status_t;
