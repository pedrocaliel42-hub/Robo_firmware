idf_component_register(
    SRCS "motion_control.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common esp_rom log robot_state stepper_a4988
)
