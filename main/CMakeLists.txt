idf_component_register(
    SRCS "main.c" "usb_desc.c" "cdc_log.c" "hid.c" "espnow.c" "encoder.c" "button.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        esp_driver_gpio
)
