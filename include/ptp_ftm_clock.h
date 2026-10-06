#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Internal transport admission, call only from the daemon's FTM tick hook.
 * A matching observation refreshes reception, not a phase-lock claim. */
bool ptpd_ftm_source_observed(int port_index, const uint8_t source_port[10],
    uint8_t domain, int8_t log_interval, int64_t received_us, uint32_t *generation);
