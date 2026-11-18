/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Built into the GPTimer driver's library (see CMakeLists.txt), so it can
 * read the driver's private handle: the software clock reads its timer's
 * counter registers directly, inside its own critical section.
 */
#include <stdbool.h>
#include "gptimer_priv.h"

bool ptp_local_timer_hw(gptimer_handle_t timer, timg_dev_t **dev, uint32_t *timer_id)
{
    if (!timer || !dev || !timer_id) return false;
    *dev = timer->hal.dev;
    *timer_id = timer->hal.timer_id;
    return *dev != NULL;
}
