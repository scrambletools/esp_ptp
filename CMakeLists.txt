idf_component_register(SRCS "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_radio_association.c" "ptp_wifi.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_timer esp_event esp_hosted_avb
                                     esp_wifi esp_driver_gptimer esp_hal_systimer
                       INCLUDE_DIRS "." "./include"
                       WHOLE_ARCHIVE)

target_sources(${COMPONENT_LIB} PRIVATE "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_radio_association.c"
               "ptp_wifi.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# The PTP compatible FTM request (one ASAP burst of three, Table 12-3) is a
# field of the initiator config, not a macro, so look for it in the header.
file(READ "${IDF_PATH}/components/esp_wifi/include/esp_wifi_types_generic.h" ptp_wifi_types)
string(FIND "${ptp_wifi_types}" "ptp_compatible" ptp_ftm_ptp_compatible)
if(NOT ptp_ftm_ptp_compatible EQUAL -1)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PTP_FTM_PTP_COMPATIBLE=1)
endif()

# ESP-Hosted peer data goes through esp_hosted_avb's hosted_avb_peer_*
# calls, which build against whichever esp-hosted the project has, so
# esp_ptp does not list espressif__esp_hosted: only projects with a
# wifi_cp port have it.

# C6 keeps the fractional SYSTIMER tick discarded by esp_timer_get_time().
idf_component_get_property(ptp_timer_dir esp_timer COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${ptp_timer_dir}/private_include")

# The C6 software clock on a hardware timer reads the timer's counter
# registers itself; this file, built into the GPTimer driver's library,
# hands it the driver's private register handle.
if(CONFIG_ESP_PTP_LOCAL_CLOCK_GPTIMER)
    idf_component_get_property(ptp_gptimer_lib esp_driver_gptimer COMPONENT_LIB)
    target_sources(${ptp_gptimer_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ptp_local_timer.c")
endif()
