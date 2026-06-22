idf_component_register(
    SRCS "mega_bridge.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common esp_driver_uart freertos log
)
