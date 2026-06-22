idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        board_config
        gripper_servo
        mega_bridge
        motion_control
        robot_state
        sensors
        serial_protocol
        stepper_a4988
)
