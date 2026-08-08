idf_component_register(
    SRCS
        "app_main.cpp"
        "cf_e714_reader.cpp"
        "keyboard_reader.cpp"
        "mqtt_publisher.cpp"
        "rfid_deduplicator.cpp"
        "time_service.cpp"
        "transport_manager.cpp"
        "yrm100_reader.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_event
        esp_eth
        esp_netif
        esp_wifi
        nvs_flash
        usb
        esp_driver_gpio
        esp_driver_uart
        esp_http_client
        app_update
        mqtt
        lwip
)

target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)

