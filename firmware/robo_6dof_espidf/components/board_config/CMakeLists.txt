idf_component_register(
    SRCS "board_config.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_common esp_driver_gpio log
)
