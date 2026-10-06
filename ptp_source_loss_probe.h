#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef CONFIG_ESP_PTP_SOURCE_LOSS_ANNOUNCE_SECONDS
#define CONFIG_ESP_PTP_SOURCE_LOSS_ANNOUNCE_SECONDS 8
#endif

/* Finite daemon-context diagnostic, never used by normal builds. */
typedef struct {
  bool started;
  int64_t start_us;
  unsigned phase;
  unsigned dropped[5];
} ptp_source_loss_probe_t;

static inline bool ptp_source_loss_probe_tick(ptp_source_loss_probe_t *probe,
                                             int64_t now_us, bool ready)
{
  if (!probe->started) {
    if (!ready || now_us < 0) return false;
    probe->started = true;
    probe->start_us = now_us;
    probe->phase = 0;
    return true;
  }
  if (now_us < probe->start_us) return false;
  int64_t elapsed = now_us - probe->start_us;
  int64_t restore = INT64_C(30000000) +
      CONFIG_ESP_PTP_SOURCE_LOSS_ANNOUNCE_SECONDS * INT64_C(1000000);
  unsigned phase = elapsed < 30000000 ? 0 : elapsed < restore ? 1 :
                   elapsed < restore + 12000000 ? 2 : elapsed < restore + 16000000 ? 3 : 4;
  if (phase == probe->phase) return false;
  probe->phase = phase;
  return true;
}

static inline bool ptp_source_loss_probe_drop(ptp_source_loss_probe_t *probe,
                                             unsigned message_type)
{
  bool drop = probe->started &&
      ((probe->phase == 1 && message_type == 11) ||
       (probe->phase == 3 && (message_type == 0 || message_type == 8)));
  if (drop) ++probe->dropped[probe->phase];
  return drop;
}
