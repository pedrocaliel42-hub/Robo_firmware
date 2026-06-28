#pragma once

#include <array>

#include "board_config.hpp"
#include "esp_err.h"

namespace robo_6dof {
namespace motion_control {

esp_err_t init();
esp_err_t move_to_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg);
esp_err_t move_home();
esp_err_t jog_relative(const std::array<float, board_config::kJointCount>& deltas_deg);

} // namespace motion_control
} // namespace robo_6dof
