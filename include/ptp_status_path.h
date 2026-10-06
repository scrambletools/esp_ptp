#pragma once
#include "esp_ptp.h"
#include <string.h>

static inline const clock_info_s *ptpd_status_source(const struct ptpd_status_s *status)
{
    return status->clock_source_selected ? &status->clock_source_info :
                                           &status->own_identity_info;
}

/* Append only observed upstream identities, never invent intermediate hops. */
static inline unsigned ptpd_status_path(const struct ptpd_status_s *status,
                                        uint8_t (*output)[8], unsigned capacity)
{
    unsigned upstream = status->clock_source_selected ? status->selected_path.count : 0;
    if (status->clock_source_selected && !upstream) return 0;
    if (!output || upstream > PTP_PATH_TRACE_MAX_CLOCKS || capacity <= upstream) return 0;
    for (unsigned index = 0; index < upstream; ++index) {
        if (!memcmp(status->selected_path.identities[index], status->own_identity_info.id, 8))
            return 0;
    }
    if (upstream) memcpy(output, status->selected_path.identities, upstream * 8);
    memcpy(output[upstream], status->own_identity_info.id, 8);
    return upstream + 1;
}
