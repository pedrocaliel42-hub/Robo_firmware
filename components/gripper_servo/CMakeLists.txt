idf_component_register(
    SRCS "gripper_servo.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_config esp_common esp_driver_ledc log
)
