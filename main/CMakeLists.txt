idf_component_register(SRCS "ble_client.c" "hid_host.c" "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES usb esp_driver_gpio bt nvs_flash
)
