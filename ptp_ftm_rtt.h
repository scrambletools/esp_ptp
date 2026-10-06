#ifndef PTP_FTM_RTT_H
#define PTP_FTM_RTT_H
#include <stdbool.h>
#include <stdint.h>

/* The driver stores small negative picosecond RTTs in an unsigned field. */
static inline bool ptp_ftm_rtt_positive(uint32_t rtt_ps)
{
    return rtt_ps > 0 && rtt_ps <= INT32_MAX;
}
#endif
