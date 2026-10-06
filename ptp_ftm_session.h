#ifndef PTP_FTM_SESSION_H
#define PTP_FTM_SESSION_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Owned by the default event loop, including session initiation. */
typedef struct {
    bool pending;
    bool valid;
    uint8_t peer[6];
} ptp_ftm_session_t;

static inline bool ptp_ftm_beacon_fresh(bool have_tsf, bool have_gptp,
    int64_t associated_us, int64_t received_us, int64_t now_us)
{
    return have_tsf && have_gptp && associated_us > 0 &&
           received_us >= associated_us && now_us >= received_us &&
           now_us - received_us <= 1000000;
}

static inline bool ptp_ftm_session_begin(ptp_ftm_session_t *session,
                                        const uint8_t peer[6])
{
    if (session->pending) return false;
    session->pending = session->valid = true;
    memcpy(session->peer, peer, 6);
    return true;
}

static inline void ptp_ftm_session_invalidate(ptp_ftm_session_t *session)
{
    session->valid = false;
}

static inline bool ptp_ftm_session_finish(ptp_ftm_session_t *session,
                                         const uint8_t peer[6], bool connected)
{
    bool accept = session->pending && session->valid && connected &&
                  memcmp(session->peer, peer, 6) == 0;
    session->pending = session->valid = false;
    return accept;
}
#endif
