idf_component_register(
    SRCS "robot_state.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common freertos log
)
