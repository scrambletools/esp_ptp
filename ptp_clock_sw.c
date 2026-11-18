/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Software PTP clock with a coherent affine anchor and relative rate control.
 * Capture each update's local time once so rate changes preserve continuity.
 */
#include "ptp.h"
#include "ptp_clock_affine.h"
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#if CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER
#include "esp_log.h"
#include "driver/gptimer.h"
#include "hal/timer_ll.h"
/* ptp_local_timer.c, built into the GPTimer driver, reads its handle. */
extern bool ptp_local_timer_hw(gptimer_handle_t timer, timg_dev_t **dev,
                               uint32_t *timer_id);
#elif CONFIG_IDF_TARGET_ESP32C6
#include "esp_timer_impl.h"
#include "hal/systimer_ll.h"
#include "soc/systimer_struct.h"
#endif

typedef int (*ptpd_sw_clock_now_fn)(struct timespec *ts);
extern void ptpd_set_sw_clock_now(ptpd_sw_clock_now_fn fn);

static ptp_clock_affine_t s_clock;
static bool s_initialized;
static portMUX_TYPE s_clock_lock = portMUX_INITIALIZER_UNLOCKED;

bool ptp_clock_sw_snapshot(ptp_clock_affine_t *snapshot)
{
    if (!snapshot) return false;
    portENTER_CRITICAL(&s_clock_lock);
    bool initialized = s_initialized;
    *snapshot = s_clock;
    portEXIT_CRITICAL(&s_clock_lock);
    return initialized;
}

#if CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER
/* A free running 40 MHz general purpose timer (PLL_F80M / 2, the
 * fastest a TIMG prescaler allows): 25 ns a tick. */
#define LOCAL_TIMER_HZ 40000000
static gptimer_handle_t s_local_timer;
static timg_dev_t *volatile s_local_dev;
static uint32_t s_local_id;
static portMUX_TYPE s_local_lock = portMUX_INITIALIZER_UNLOCKED;

static int local_timer_start(void)
{
    if (s_local_dev) return 0;
    gptimer_config_t config = {.clk_src = GPTIMER_CLK_SRC_PLL_F80M,
                               .direction = GPTIMER_COUNT_UP,
                               .resolution_hz = LOCAL_TIMER_HZ};
    esp_err_t err = s_local_timer ? ESP_OK : gptimer_new_timer(&config, &s_local_timer);
    if (err == ESP_OK) err = gptimer_enable(s_local_timer);
    if (err == ESP_OK) err = gptimer_start(s_local_timer);
    timg_dev_t *dev = NULL;
    uint32_t id = 0;
    if (err == ESP_OK && !ptp_local_timer_hw(s_local_timer, &dev, &id)) err = ESP_FAIL;
    if (err != ESP_OK) {
        ESP_LOGE("ptp_clock", "local clock timer: %s", esp_err_to_name(err));
        return -1;
    }
    s_local_id = id;
    s_local_dev = dev;
    ESP_LOGI("ptp_clock", "local clock: 40 MHz hardware timer, 25 ns a tick");
    return 0;
}

void ptp_clock_local_latch(void)
{
    if (s_local_dev) timer_ll_trigger_soft_capture(s_local_dev, s_local_id);
}

int64_t ptp_clock_local_latched_ns(void)
{
    if (!s_local_dev) return 0;
    return (int64_t)timer_ll_get_counter_value(s_local_dev, s_local_id) * 25;
}

int64_t ptp_clock_local_ns(void)
{
    if (!s_local_dev) return 0;
    portENTER_CRITICAL_SAFE(&s_local_lock);
    timer_ll_trigger_soft_capture(s_local_dev, s_local_id);
    uint64_t ticks = timer_ll_get_counter_value(s_local_dev, s_local_id);
    portEXIT_CRITICAL_SAFE(&s_local_lock);
    return (int64_t)ticks * 25;
}
#elif CONFIG_IDF_TARGET_ESP32C6
/* C6 SYSTIMER uses the 40 MHz crystal divided by 2.5, or 62.5 ns/tick. */
static inline int64_t systimer_ns(uint64_t ticks)
{
    return (int64_t)(ticks / 2 * 125 + (ticks & 1) * 62);
}

void ptp_clock_local_latch(void)
{
    systimer_ll_counter_snapshot(&SYSTIMER, 0);
    for (unsigned attempts = 0;
         !systimer_ll_is_counter_value_valid(&SYSTIMER, 0) && attempts < 64; ++attempts) {}
}

int64_t ptp_clock_local_latched_ns(void)
{
    uint32_t low = systimer_ll_get_counter_value_low(&SYSTIMER, 0);
    uint32_t high = systimer_ll_get_counter_value_high(&SYSTIMER, 0);
    return systimer_ns(((uint64_t)high << 32) | low);
}

int64_t ptp_clock_local_ns(void)
{
    return systimer_ns(esp_timer_impl_get_counter_reg());
}
#else
void ptp_clock_local_latch(void) {}

int64_t ptp_clock_local_latched_ns(void)
{
    return esp_timer_get_time() * 1000;
}

int64_t ptp_clock_local_ns(void)
{
    return esp_timer_get_time() * 1000;
}
#endif

static int sw_now_timespec(struct timespec *ts)
{
    if (!ts) { errno = EFAULT; return -1; }
    ptp_clock_affine_t snapshot;
    int64_t local_ns;
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL;
        return -1;
    }
    snapshot = s_clock;
    local_ns = ptp_clock_local_ns();
    portEXIT_CRITICAL(&s_clock_lock);
    int64_t now_ns = ptp_clock_affine_now(&snapshot, local_ns);
    ts->tv_sec = (time_t)(now_ns / 1000000000LL);
    ts->tv_nsec = (long)(now_ns % 1000000000LL);
    if (ts->tv_nsec < 0) { --ts->tv_sec; ts->tv_nsec += 1000000000L; }
    return 0;
}

int ptp_clock_sw_init(const struct timespec *initial_ts)
{
#if CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER
    if (local_timer_start() != 0) return -1;
#endif
    int64_t initial_ns = initial_ts ?
        (int64_t)initial_ts->tv_sec * 1000000000LL + initial_ts->tv_nsec : 0;
    portENTER_CRITICAL(&s_clock_lock);
    s_clock = (ptp_clock_affine_t){.local_ns = ptp_clock_local_ns(),
                                  .ptp_ns = initial_ns};
    s_initialized = true;
    portEXIT_CRITICAL(&s_clock_lock);
    ptpd_set_sw_clock_now(sw_now_timespec);
    return 0;
}

int ptp_clock_sw_now(struct timespec *ts)
{
    return sw_now_timespec(ts);
}

int ptp_clock_sw_settime(const struct timespec *ts)
{
    if (!ts) { errno = EINVAL; return -1; }
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    s_clock.local_ns = ptp_clock_local_ns();
    s_clock.ptp_ns = (int64_t)ts->tv_sec * 1000000000LL + ts->tv_nsec;
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_adjtime_offset(int64_t delta_ns)
{
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    ptp_clock_affine_reanchor(&s_clock, ptp_clock_local_ns(), delta_ns);
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_adjtime_rate(int32_t rate_ppb)
{
    portENTER_CRITICAL(&s_clock_lock);
    if (!s_initialized) {
        portEXIT_CRITICAL(&s_clock_lock);
        errno = EINVAL; return -1;
    }
    ptp_clock_affine_adjust_rate(&s_clock, ptp_clock_local_ns(), rate_ppb);
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}

int ptp_clock_sw_discipline(int64_t local_anchor, int64_t reference_anchor,
    int32_t rate_ppb, bool step, int64_t *phase_error, int32_t *applied_rate)
{
    if (!phase_error || !applied_rate || local_anchor < 0 || reference_anchor < 0 ||
        reference_anchor > INT64_MAX - INT64_C(3000000000) ||
        rate_ppb < -200000 || rate_ppb > 200000) return -1;
    portENTER_CRITICAL(&s_clock_lock);
    int64_t now_ns = ptp_clock_local_ns();
    if (!s_initialized || now_ns < local_anchor || now_ns - local_anchor > INT64_C(2000000000)) {
        portEXIT_CRITICAL(&s_clock_lock);
        return -1;
    }
    ptp_clock_affine_discipline(&s_clock, now_ns, local_anchor, reference_anchor,
        rate_ppb, step, phase_error);
    *applied_rate = s_clock.rate_ppb;
    portEXIT_CRITICAL(&s_clock_lock);
    return 0;
}
