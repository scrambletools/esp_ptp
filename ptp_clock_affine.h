#ifndef PTP_CLOCK_AFFINE_H
#define PTP_CLOCK_AFFINE_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int64_t local_ns;
    int64_t ptp_ns;
    int32_t rate_ppb;
} ptp_clock_affine_t;

/* Coherent task-context snapshot, for timestamp conversion outside read brackets. */
bool ptp_clock_sw_snapshot(ptp_clock_affine_t *snapshot);
int ptp_clock_sw_discipline(int64_t local_anchor, int64_t reference_anchor,
    int32_t rate_ppb, bool step, int64_t *phase_error, int32_t *applied_rate);

static inline int64_t ptp_clock_rate_correction(int64_t delta_ns, int32_t rate_ppb)
{
    /* Split the product so long holdover does not overflow at ordinary rates. */
    return (delta_ns / INT64_C(1000000000)) * rate_ppb +
           (delta_ns % INT64_C(1000000000)) * rate_ppb / INT64_C(1000000000);
}

static inline int64_t ptp_clock_affine_now(const ptp_clock_affine_t *clock,
                                          int64_t local_ns)
{
    int64_t delta = local_ns - clock->local_ns;
    return clock->ptp_ns + delta + ptp_clock_rate_correction(delta, clock->rate_ppb);
}

static inline void ptp_clock_affine_reanchor(ptp_clock_affine_t *clock,
                                            int64_t local_ns, int64_t offset_ns)
{
    clock->ptp_ns = ptp_clock_affine_now(clock, local_ns) + offset_ns;
    clock->local_ns = local_ns;
}

static inline void ptp_clock_affine_adjust_rate(ptp_clock_affine_t *clock,
                                               int64_t local_ns, int32_t adjustment)
{
    ptp_clock_affine_reanchor(clock, local_ns, 0);
    int64_t rate = clock->rate_ppb;
    rate += rate * adjustment / INT64_C(1000000000) + adjustment;
    if (rate > 100000000) rate = 100000000;
    if (rate < -100000000) rate = -100000000;
    clock->rate_ppb = (int32_t)rate;
}

static inline void ptp_clock_affine_discipline(ptp_clock_affine_t *clock,
    int64_t now_ns, int64_t local_anchor, int64_t reference_anchor,
    int32_t rate_ppb, bool step, int64_t *phase_error)
{
    int64_t elapsed = now_ns - local_anchor;
    int64_t target = reference_anchor + elapsed + ptp_clock_rate_correction(elapsed, rate_ppb);
    int64_t actual = ptp_clock_affine_now(clock, now_ns);
    *phase_error = target - actual;
    int64_t phase_trim = *phase_error;
    if (phase_trim > 200000) phase_trim = 200000;
    if (phase_trim < -200000) phase_trim = -200000;
    int64_t applied = step ? rate_ppb : rate_ppb + phase_trim;
    if (applied > 512000) applied = 512000;
    if (applied < -512000) applied = -512000;
    clock->local_ns = now_ns;
    clock->ptp_ns = step ? target : actual;
    clock->rate_ppb = (int32_t)applied;
}
#endif
