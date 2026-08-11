idf_component_register(
    SRCS
        "app_main.cpp"
        "cf_e714_reader.cpp"
        "config_portal.cpp"
        "keyboard_reader.cpp"
        "mqtt_publisher.cpp"
        "rfid_deduplicator.cpp"
        "runtime_settings.cpp"
        "status_lcd.cpp"
        "time_service.cpp"
        "transport_manager.cpp"
        "yrm100_reader.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_event
        esp_eth
        esp_netif
        esp_wifi
        network_provisioning
        nvs_flash
        usb
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_uart
        esp_http_client
        esp_https_server
        app_update
        mqtt
        lwip
        esp_modem
    EMBED_TXTFILES
        "certs/servercert.pem"
        "certs/prvtkey.pem"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)

