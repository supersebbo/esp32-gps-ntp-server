set(SRCS_LIST
    "main.c"
    "gps.c"
    "ntp_server.c"
    "sntp_fallback.c"
)

set(REQUIRES_LIST
    esp_netif
    esp_event
    nvs_flash
    driver
    esp_timer
    lwip
    newlib
    log
    freertos
    esp_wifi
)

list(APPEND SRCS_LIST     "ethernet.c")
list(APPEND REQUIRES_LIST esp_eth)

list(APPEND SRCS_LIST "i2c_bus.c")
list(APPEND SRCS_LIST "oled.c")
list(APPEND SRCS_LIST "rtc.c")
list(APPEND SRCS_LIST "ota_server.c")
list(APPEND SRCS_LIST "log_server.c")
list(APPEND REQUIRES_LIST app_update esp_http_server)

idf_component_register(
    SRCS          ${SRCS_LIST}
    INCLUDE_DIRS  "."
    REQUIRES      ${REQUIRES_LIST}
)
