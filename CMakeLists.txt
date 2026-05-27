idf_component_register(SRCS "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_wifi.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_timer esp_event esp_ptp_rpc esp_wifi
                       INCLUDE_DIRS "." "./include"
                       WHOLE_ARCHIVE)

target_sources(${COMPONENT_LIB} PRIVATE "ptp.c" "ptp_clock_sw.c" "ptp_beacon_ie.c" "ptp_wifi.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)

# Host-side ESP-Hosted symbols are referenced via forward declarations
# in ptp_beacon_ie.c (see the comment block there). We don't list
# espressif__esp_hosted in PRIV_REQUIRES because it's not present in
# all projects using esp_ptp — only ones with a wifi_cp port have it.
# The link succeeds because those builds pull espressif__esp_hosted
# in transitively (the host application uses esp_wifi_remote, which
# depends on esp_hosted).
