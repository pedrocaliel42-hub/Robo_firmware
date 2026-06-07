idf_component_register(
    SRCS "stepper_a4988.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common esp_driver_gpio esp_rom log
)
