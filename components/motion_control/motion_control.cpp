#include "motion_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "board_config.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "robot_state.hpp"
#include "stepper_a4988.hpp"

namespace {
constexpr char TAG[] = "motion_control";
constexpr uint32_t kStepIntervalUs = 2000;

float steps_per_degree(const robo_6dof::board_config::JointConfig& joint)
{
    return (static_cast<float>(joint.steps_per_rev) *
            static_cast<float>(joint.microstep) *
            joint.gear_ratio) /
           360.0F;
}

int32_t degrees_to_steps(std::size_t axis, float degrees)
{
    const auto* joint = robo_6dof::board_config::joint_config(axis);
    if (joint == nullptr) {
        return 0;
    }

    return static_cast<int32_t>(std::lround(degrees * steps_per_degree(*joint)));
}

bool targets_in_limits(const std::array<float, robo_6dof::board_config::kJointCount>& targets_deg)
{
    return robo_6dof::board_config::validate_joint_targets_deg(
        targets_deg.data(),
        targets_deg.size());
}

esp_err_t execute_blocking_move(
    const std::array<float, robo_6dof::board_config::kJointCount>& current_deg,
    const std::array<float, robo_6dof::board_config::kJointCount>& targets_deg)
{
    std::array<int32_t, robo_6dof::board_config::kJointCount> delta_steps = {};
    std::array<uint32_t, robo_6dof::board_config::kJointCount> abs_steps = {};
    std::array<uint32_t, robo_6dof::board_config::kJointCount> accumulators = {};

    uint32_t max_steps = 0;

    for (std::size_t axis = 0; axis < delta_steps.size(); ++axis) {
        const int32_t current_steps = degrees_to_steps(axis, current_deg[axis]);
        const int32_t target_steps = degrees_to_steps(axis, targets_deg[axis]);
        delta_steps[axis] = target_steps - current_steps;
        abs_steps[axis] = static_cast<uint32_t>(std::abs(delta_steps[axis]));
        max_steps = std::max(max_steps, abs_steps[axis]);

        ESP_ERROR_CHECK(robo_6dof::stepper_a4988::set_direction(axis, delta_steps[axis] >= 0));
    }

    if (max_steps == 0) {
        ESP_LOGI(TAG, "no stepper movement required");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "starting coordinated move, max_steps=%u", static_cast<unsigned>(max_steps));

    for (uint32_t step_index = 0; step_index < max_steps; ++step_index) {
        if (robo_6dof::robot_state::is_estop() ||
            robo_6dof::robot_state::mode() != robo_6dof::robot_state::RobotMode::Moving) {
            robo_6dof::stepper_a4988::emergency_stop();
            ESP_LOGW(TAG, "move interrupted at step %u/%u",
                     static_cast<unsigned>(step_index),
                     static_cast<unsigned>(max_steps));
            return ESP_ERR_INVALID_STATE;
        }

        for (std::size_t axis = 0; axis < abs_steps.size(); ++axis) {
            if (abs_steps[axis] == 0) {
                continue;
            }

            accumulators[axis] += abs_steps[axis];
            if (accumulators[axis] >= max_steps) {
                accumulators[axis] -= max_steps;
                ESP_ERROR_CHECK(robo_6dof::stepper_a4988::pulse(axis));
            }
        }

        esp_rom_delay_us(kStepIntervalUs);
    }

    ESP_LOGI(TAG, "coordinated move complete");
    return ESP_OK;
}
} // namespace

namespace robo_6dof {
namespace motion_control {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

esp_err_t move_to_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg)
{
    if (!targets_in_limits(targets_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    const auto current = robot_state::snapshot();
    if (robot_state::begin_motion() != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t move_err = execute_blocking_move(current.joints_deg, targets_deg);
    if (move_err != ESP_OK) {
        return move_err;
    }

    return robot_state::finish_joint_motion(targets_deg);
}

esp_err_t move_home()
{
    std::array<float, board_config::kJointCount> targets = {};
    for (std::size_t axis = 0; axis < targets.size(); ++axis) {
        const auto* joint = board_config::joint_config(axis);
        if (joint == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        targets[axis] = joint->home_deg;
    }

    return move_to_targets_deg(targets);
}

} // namespace motion_control
} // namespace robo_6dof
