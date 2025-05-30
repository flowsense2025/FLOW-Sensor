idf_component_register(SRCS "user_spi_interface.c" "app_main.c" "gatt_svr.c" "ble_server.cpp" "user_spi_interface.c"
                    PRIV_REQUIRES spi_flash bt nvs_flash
                    REQUIRES driver
                    INCLUDE_DIRS "")

add_compile_definitions(BLE_42_FEATURE_SUPPORT)