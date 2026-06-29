#pragma once

#include <array>

#include "board_config.hpp"
#include "esp_err.h"

namespace robo_6dof {
namespace mega_bridge {

esp_err_t init();
esp_err_t ping();
esp_err_t reference_home();
esp_err_t set_q23_power(bool enabled);
esp_err_t set_q2_locked(bool locked);
esp_err_t prepare_move(const std::array<float, board_config::kMegaJointCount>& targets_deg);
esp_err_t start_prepared_move();
esp_err_t wait_for_move_done(uint32_t timeout_ms = 0);
esp_err_t stop();
esp_err_t emergency_stop();
esp_err_t set_gripper_percent(int percent);
esp_err_t jog_relative(
    const std::array<float, board_config::kMegaJointCount>& deltas_deg,
    uint32_t timeout_ms = 0);

} // namespace mega_bridge
} // namespace robo_6dof
