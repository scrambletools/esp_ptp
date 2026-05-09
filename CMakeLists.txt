idf_component_register(SRCS "ptpd.c" "ptp_clock_sw.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_timer
                       INCLUDE_DIRS "." "./include")

target_sources(${COMPONENT_LIB} PRIVATE "ptpd.c" "ptp_clock_sw.c")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
