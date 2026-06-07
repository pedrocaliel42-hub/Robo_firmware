#pragma once

#include <array>

#include "board_config.hpp"
#include "esp_err.h"

namespace robo_6dof {
namespace robot_state {

struct Snapshot {
    std::array<float, board_config::kJointCount> joints_deg;
    int gripper_percent;
    bool armed;
    bool estop;
};

esp_err_t init();
esp_err_t start();
esp_err_t stop();
esp_err_t emergency_stop();
bool is_armed();
bool is_estop();
esp_err_t set_joint_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg);
esp_err_t set_home_position();
esp_err_t set_gripper_percent(int percent);
Snapshot snapshot();

} // namespace robot_state
} // namespace robo_6dof
