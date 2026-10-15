idf_component_register(SRCS "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_radio_association.c" "ptp_wifi.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_timer esp_event esp_hosted_avb
                                     esp_wifi
                       INCLUDE_DIRS "." "./include"
                       WHOLE_ARCHIVE)

target_sources(${COMPONENT_LIB} PRIVATE "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_radio_association.c"
               "ptp_wifi.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# ESP-Hosted peer data goes through esp_hosted_avb's hosted_avb_peer_*
# calls, which build against whichever esp-hosted the project has, so
# esp_ptp does not list espressif__esp_hosted: only projects with a
# wifi_cp port have it.

# C6 keeps the fractional SYSTIMER tick discarded by esp_timer_get_time().
idf_component_get_property(ptp_timer_dir esp_timer COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${ptp_timer_dir}/private_include")
