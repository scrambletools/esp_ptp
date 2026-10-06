#pragma once
#include <stdbool.h>
#include <stdint.h>

/* IEEE 802.1AS-2020 12.3 and 12.4: media capability of one IEEE 802.11
 * association. Timing Measurement and Fine Timing Measurement support are
 * established from this port's own support and the peer's Extended
 * Capabilities. granted_frames is the FTMs-per-burst value the responder
 * granted for the current initial request, 0 while nothing is granted. */
typedef struct {
    bool tm_local, tm_peer;
    bool ftm_local, ftm_peer;
    uint8_t granted_frames;
} ptp_wifi_media_t;

/* Table 12-1: bit 0 Timing Measurement, bit 1 Fine Timing Measurement. */
static inline uint8_t ptp_wifi_tm_ftm_support(const ptp_wifi_media_t *media)
{
    return (uint8_t)((media->tm_local && media->tm_peer ? 1 : 0) |
                     (media->ftm_local && media->ftm_peer ? 2 : 0));
}

/* 12.4: asCapable requires nonzero tmFtmSupport, neighborGptpCapable and
 * either TM support or an FTM grant of three or two frames per burst. A
 * domain 0 port with TM support may interoperate with a 2011 neighbor that
 * never signals capability. Nothing here inspects clock readiness. */
static inline bool ptp_wifi_as_capable(const ptp_wifi_media_t *media,
                                       bool neighbor_gptp_capable, unsigned domain)
{
    uint8_t support = ptp_wifi_tm_ftm_support(media);
    if (!support) return false;
    bool timing_measurement = support & 1;
    bool ftm_granted = (support & 2) &&
        (media->granted_frames == 3 || media->granted_frames == 2);
    if (neighbor_gptp_capable && (timing_measurement || ftm_granted)) return true;
    return domain == 0 && timing_measurement;
}
