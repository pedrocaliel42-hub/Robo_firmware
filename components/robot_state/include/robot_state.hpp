#pragma once

#include <array>

#include "board_config.hpp"
#include "esp_err.h"

namespace robo_6dof {
namespace robot_state {

enum class RobotMode {
    Boot,
    Idle,
    Armed,
    Moving,
    Stopped,
    Estop,
    Fault,
};

struct Snapshot {
    std::array<float, board_config::kJointCount> joints_deg;
    int gripper_percent;
    RobotMode mode;
};

esp_err_t init();
esp_err_t start();
esp_err_t stop();
esp_err_t emergency_stop();
bool is_armed();
bool is_estop();
bool can_accept_motion();
RobotMode mode();
const char* mode_name(RobotMode mode);
esp_err_t begin_motion();
esp_err_t finish_jog();
esp_err_t finish_joint_motion(const std::array<float, board_config::kJointCount>& reached_deg);
esp_err_t finish_home_motion();
esp_err_t set_joint_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg);
esp_err_t set_home_position();
esp_err_t set_gripper_percent(int percent);
Snapshot snapshot();

} // namespace robot_state
} // namespace robo_6dof
