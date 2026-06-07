#include <cstdio>

#include "board_config.hpp"
#include "gripper_servo.hpp"
#include "motion_control.hpp"
#include "robot_state.hpp"
#include "sensors.hpp"
#include "serial_protocol.hpp"
#include "stepper_a4988.hpp"

#include "esp_err.h"
#include "esp_log.h"

namespace {
constexpr char TAG[] = "robo_6dof";

void configure_default_console()
{
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);
}
} // namespace

extern "C" void app_main(void)
{
    configure_default_console();

    ESP_LOGI(TAG, "Starting ROBO_6DOF ESP-IDF firmware foundation");

    ESP_ERROR_CHECK(robo_6dof::board_config::init());
    ESP_ERROR_CHECK(robo_6dof::robot_state::init());
    ESP_ERROR_CHECK(robo_6dof::sensors::init());
    ESP_ERROR_CHECK(robo_6dof::stepper_a4988::init());
    ESP_ERROR_CHECK(robo_6dof::gripper_servo::init());
    ESP_ERROR_CHECK(robo_6dof::motion_control::init());
    ESP_ERROR_CHECK(robo_6dof::serial_protocol::init());

    ESP_ERROR_CHECK(robo_6dof::serial_protocol::send_boot_message());
    ESP_ERROR_CHECK(robo_6dof::serial_protocol::start());

    ESP_LOGI(TAG, "Firmware initialization complete");
}
