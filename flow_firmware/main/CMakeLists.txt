idf_component_register(SRCS "gatt_svr.c" "ble_server.cpp" "app_main.cpp"
                    PRIV_REQUIRES spi_flash bt nvs_flash
                    REQUIRES driver
                    INCLUDE_DIRS "")

add_compile_definitions(BLE_42_FEATURE_SUPPORT)