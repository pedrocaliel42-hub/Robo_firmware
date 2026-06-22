#pragma once

#include <cstddef>

#include "driver/gpio.h"
#include "esp_err.h"

namespace robo_6dof {
namespace board_config {

constexpr std::size_t kJointCount = 6;
constexpr std::size_t kMegaJointCount = 5;
constexpr std::size_t kLocalStepperAxis = 5; // q6

struct HardwarePins {
    gpio_num_t start_button_gpio;
    gpio_num_t stop_estop_button_gpio;
    gpio_num_t limit_switch_gpio;
    gpio_num_t gripper_servo_gpio;
    gpio_num_t mega_uart_rx_gpio;
    gpio_num_t mega_uart_tx_gpio;
};

struct JointConfig {
    const char* name;
    gpio_num_t step_gpio;
    gpio_num_t dir_gpio;
    float min_deg;
    float max_deg;
    float home_deg;
    int steps_per_rev;
    int microstep;
    float gear_ratio;
    bool invert_dir;
    float max_speed_dps;
    float max_accel_dps2;
};

esp_err_t init();
const HardwarePins& hardware_pins();
const JointConfig* joint_config(std::size_t axis);
const JointConfig* joint_configs();
std::size_t joint_count();
bool is_mega_axis(std::size_t axis);
bool is_local_stepper_axis(std::size_t axis);
bool validate_joint_target_deg(std::size_t axis, float target_deg);
bool validate_joint_targets_deg(
    const float* targets_deg,
    std::size_t count,
    std::size_t* first_invalid_axis = nullptr);

} // namespace board_config
} // namespace robo_6dof
