#pragma once
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

/* Rate is remote clock units per local clock unit. */
typedef struct {
    uint32_t lifecycle;
    uint8_t responder[10];
    bool anchored, valid;
    struct timespec remote_previous, local_previous;
    double correction_previous, ratio;
    unsigned updates;
} ptp_peer_rate_t;

static inline double ptp_peer_correction(const uint8_t wire[8])
{
    uint64_t scaled = 0;
    for (unsigned index = 0; index < 8; ++index) scaled = (scaled << 8) | wire[index];
    return (double)(int64_t)scaled / 65536.0;
}

static inline bool ptp_peer_elapsed(const struct timespec *current,
    const struct timespec *previous, double *elapsed)
{
    if (current->tv_sec < previous->tv_sec ||
        current->tv_sec - previous->tv_sec > 10) return false;
    *elapsed = (double)(current->tv_sec - previous->tv_sec) * 1e9 +
        current->tv_nsec - previous->tv_nsec;
    return *elapsed >= 1e7 && *elapsed <= 1e10;
}

static inline bool ptp_peer_rate_update(ptp_peer_rate_t *rate,
    uint32_t lifecycle, const uint8_t responder[10],
    const struct timespec *remote, double correction,
    const struct timespec *local)
{
    if (rate->lifecycle != lifecycle || memcmp(rate->responder, responder, 10)) {
        memset(rate, 0, sizeof(*rate));
        rate->lifecycle = lifecycle;
        memcpy(rate->responder, responder, 10);
    }
    double local_elapsed, remote_elapsed;
    bool valid = rate->anchored &&
        ptp_peer_elapsed(local, &rate->local_previous, &local_elapsed) &&
        ptp_peer_elapsed(remote, &rate->remote_previous, &remote_elapsed);
    double ratio = valid ? (remote_elapsed + correction -
        rate->correction_previous) / local_elapsed : 0;
    valid = valid && isfinite(ratio) && ratio >= .999 && ratio <= 1.001;
    rate->remote_previous = *remote;
    rate->local_previous = *local;
    rate->correction_previous = correction;
    rate->anchored = true;
    rate->valid = valid;
    rate->ratio = valid ? ratio : 0;
    if (valid) ++rate->updates;
    return valid;
}

/* Convert remote turnaround to local units before removing it from RTT. */
static inline bool ptp_peer_delay_local(double ratio, int64_t roundtrip,
    int64_t reflection, double response_correction, double follow_correction,
    double *delay)
{
    if (!isfinite(ratio) || ratio < .999 || ratio > 1.001 ||
        roundtrip < 0 || roundtrip > 1000000000LL ||
        reflection < 0 || reflection > 1000000000LL) return false;
    *delay = ((double)roundtrip - ((double)reflection + follow_correction -
        response_correction) / ratio) / 2;
    return isfinite(*delay) && *delay >= 0;
}
