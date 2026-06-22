idf_component_register(
    SRCS "serial_protocol.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common freertos gripper_servo log mega_bridge motion_control robot_state
)
