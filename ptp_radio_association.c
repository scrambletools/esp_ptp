/* SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 Scramble Tools
 *
 * Host side of the coprocessor's association reports (esp_hosted_avb
 * HOSTED_AVB_MSG_ASSOCIATIONS). The AP port sends a station Announce and
 * gPTP-capable Signaling only once its peers are reconciled to the
 * radio's associations, which these reports keep current. A report
 * comes in behind the association event the coprocessor forwarded
 * before it and crosses the default event loop, so it is reconciled
 * after that event's join or leave.
 */

#include "sdkconfig.h"

#ifdef CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR

#include "ptp.h"
#include "hosted_avb_proto.h"
#include "ptp_wifi_association.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward-declared, as in ptp_beacon_ie.c, so esp_ptp needs no hard
 * dependency on espressif__esp_hosted. */
extern esp_err_t esp_hosted_send_custom_data(uint32_t msg_id,
                                             const uint8_t *data,
                                             size_t data_len);
extern esp_err_t esp_hosted_register_custom_callback(
    uint32_t msg_id, void (*cb)(uint32_t, const uint8_t *, size_t, void *),
    void *user);

static const char *TAG = "ptp_radio_assoc";

ESP_EVENT_DEFINE_BASE(PTP_RADIO_ASSOCIATION_EVENT);

/* The host asks for a report this often, so a lost one costs at most an
 * interval; a coprocessor that has not answered the first request does
 * not report at all. */
#define RADIO_ASSOCIATION_REFRESH_MS 5000

static int s_port_index = -1;
static volatile bool s_any_report;
static uint32_t s_radio_boot;
static uint32_t s_logged_generation;

static void on_associations(uint32_t msg_id, const uint8_t *data,
                            size_t data_len, void *user) {
  (void)msg_id;
  (void)user;
  if (data == NULL || data_len != sizeof(hosted_avb_associations_t)) {
    return;
  }
  const hosted_avb_associations_t *report =
      (const hosted_avb_associations_t *)data;
  if (report->version != HOSTED_AVB_ASSOCIATIONS_VERSION ||
      report->count > HOSTED_AVB_ASSOCIATIONS_MAX) {
    return;
  }
  /* A full event queue drops the report; the next refresh repeats it. */
  (void)esp_event_post(PTP_RADIO_ASSOCIATION_EVENT, 0, report,
                       sizeof(*report), 0);
}

static void reconcile_associations(void *arg, esp_event_base_t base,
                                   int32_t id, void *data) {
  (void)arg;
  (void)base;
  (void)id;
  const hosted_avb_associations_t *report = data;
  if (s_port_index < 0 || report == NULL) {
    return;
  }
  s_any_report = true;
  bool new_boot = report->radio_boot != s_radio_boot;
  s_radio_boot = report->radio_boot;
  if (!(report->flags & HOSTED_AVB_ASSOCIATIONS_FLAG_LIFECYCLE)) {
    /* Not reporting changes yet, as before its first association, or
     * without the lifecycle patch: its list cannot be trusted. */
    if (new_boot) {
      ESP_LOGI(TAG,
               "Coprocessor %08lx has not reported an association change; "
               "serving stations from the host's association events",
               (unsigned long)report->radio_boot);
      ptpd_wifi_association_unreported(s_port_index);
    }
    return;
  }
  bool accepted = ptpd_wifi_association_reconcile(
      s_port_index, report->radio_boot, report->generation, report->macs,
      report->count);
  if (accepted && (new_boot || report->generation != s_logged_generation)) {
    s_logged_generation = report->generation;
    ESP_LOGI(TAG, "Coprocessor %08lx associations, generation %lu: %u",
             (unsigned long)report->radio_boot,
             (unsigned long)report->generation, (unsigned)report->count);
  }
}

static void request_associations(void) {
  static const uint8_t request = HOSTED_AVB_ASSOCIATIONS_VERSION;
  static esp_err_t s_last_result = ESP_OK;
  esp_err_t result = esp_hosted_send_custom_data(
      HOSTED_AVB_MSG_ASSOCIATIONS_REQ, &request, sizeof(request));
  /* Logged once a failure starts; an older coprocessor fails every time. */
  if (result != ESP_OK && result != s_last_result) {
    ESP_LOGW(TAG, "ASSOCIATIONS_REQ send failed: %s", esp_err_to_name(result));
  }
  s_last_result = result;
}

/* Its own task: a custom RPC send blocks on the SDIO round trip. */
static void refresh_task(void *arg) {
  (void)arg;
  bool warned = false;
  for (;;) {
    request_associations();
    vTaskDelay(pdMS_TO_TICKS(RADIO_ASSOCIATION_REFRESH_MS));
    if (!s_any_report && !warned) {
      ESP_LOGW(TAG,
               "Coprocessor does not report its associations (esp_ptp_rpc "
               "before 0.3.0); serving stations from the host's association "
               "events");
      ptpd_wifi_association_unreported(s_port_index);
      warned = true;
    }
  }
}

int ptp_radio_association_attach(int port_index) {
  if (s_port_index >= 0) {
    return s_port_index == port_index ? 0 : -EBUSY;
  }
  if (esp_event_handler_register(PTP_RADIO_ASSOCIATION_EVENT,
                                 ESP_EVENT_ANY_ID, reconcile_associations,
                                 NULL) != ESP_OK ||
      esp_hosted_register_custom_callback(HOSTED_AVB_MSG_ASSOCIATIONS,
                                          on_associations, NULL) != ESP_OK) {
    ESP_LOGE(TAG, "failed to register for association reports");
    return -EIO;
  }
  s_port_index = port_index;
  if (xTaskCreate(refresh_task, "ptp_assoc", 3072, NULL, 2, NULL) != pdPASS) {
    ESP_LOGE(TAG, "failed to create the association refresh task");
    return -ENOMEM;
  }
  return 0;
}

#endif /* CONFIG_ESP_PTP_HAS_AP_VIA_COPROCESSOR */
