# USV GPS/RTK module simulator — application component.

idf_component_register(
    SRCS
        "main.c"
        "sim.c"
        "nmea.c"
        "rtcm.c"
        "web.c"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        nvs_flash
        esp_driver_uart
        esp_driver_gpio
        esp_http_server
        lwip
)
