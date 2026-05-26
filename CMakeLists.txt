set(esp_ptp_private_requires esp_eth esp_netif esp_timer esp_event esp_wifi)
if(CONFIG_ESP_PTP_HAS_WIFI_CP_AP)
    list(APPEND esp_ptp_private_requires esp_ptp_rpc)
endif()

idf_component_register(SRCS "ptpd.c" "ptp_clock_sw.c" "ptp_beacon_ie.c"
                       PRIV_REQUIRES ${esp_ptp_private_requires}
                       INCLUDE_DIRS "." "./include"
                       WHOLE_ARCHIVE)

target_sources(${COMPONENT_LIB} PRIVATE "ptpd.c" "ptp_clock_sw.c" "ptp_beacon_ie.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# Host-side ESP-Hosted symbols are referenced via forward declarations
# in ptp_beacon_ie.c (see the comment block there). We don't list
# espressif__esp_hosted in PRIV_REQUIRES because it's not present in
# all projects using esp_ptp — only ones with a wifi_cp port have it.
# The link succeeds because those builds pull espressif__esp_hosted
# in transitively (the host application uses esp_wifi_remote, which
# depends on esp_hosted).
