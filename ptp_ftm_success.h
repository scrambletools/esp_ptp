#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Percent of FTM measurements valid over the last ten seconds, kept in
 * one-second buckets (profiles/avb_wireless.md §5.1 ftm_success). The
 * caller serializes access. */
#define PTP_FTM_SUCCESS_SECONDS 10
#define PTP_FTM_SUCCESS_NONE 0xFF

typedef struct {
    int64_t second[PTP_FTM_SUCCESS_SECONDS];
    uint16_t attempts[PTP_FTM_SUCCESS_SECONDS];
    uint16_t valid[PTP_FTM_SUCCESS_SECONDS];
} ptp_ftm_success_t;

static inline void ptp_ftm_success_record(ptp_ftm_success_t *window,
                                          int64_t now_us, bool valid)
{
    int64_t second = now_us / 1000000;
    unsigned slot = (unsigned)(second % PTP_FTM_SUCCESS_SECONDS);
    if (window->second[slot] != second) {
        window->second[slot] = second;
        window->attempts[slot] = window->valid[slot] = 0;
    }
    if (window->attempts[slot] == UINT16_MAX) return;
    ++window->attempts[slot];
    if (valid) ++window->valid[slot];
}

/* PTP_FTM_SUCCESS_NONE when no measurement was attempted in the window. */
static inline uint8_t ptp_ftm_success_percent(const ptp_ftm_success_t *window,
                                              int64_t now_us)
{
    int64_t second = now_us / 1000000;
    uint32_t attempts = 0, valid = 0;
    for (unsigned slot = 0; slot < PTP_FTM_SUCCESS_SECONDS; ++slot) {
        int64_t age = second - window->second[slot];
        if (age < 0 || age >= PTP_FTM_SUCCESS_SECONDS) continue;
        attempts += window->attempts[slot];
        valid += window->valid[slot];
    }
    if (!attempts) return PTP_FTM_SUCCESS_NONE;
    return (uint8_t)((valid * 100 + attempts / 2) / attempts);
}
