#include "motion_control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "board_config.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "mega_bridge.hpp"
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
    constexpr std::size_t axis = robo_6dof::board_config::kLocalStepperAxis;

    const int32_t current_steps = degrees_to_steps(axis, current_deg[axis]);
    const int32_t target_steps = degrees_to_steps(axis, targets_deg[axis]);
    const int32_t delta_steps = target_steps - current_steps;
    const uint32_t abs_steps = static_cast<uint32_t>(std::abs(delta_steps));

    ESP_RETURN_ON_ERROR(
        robo_6dof::stepper_a4988::set_direction(axis, delta_steps >= 0),
        TAG,
        "failed to set q6 direction");

    if (abs_steps == 0) {
        ESP_LOGD(TAG, "no local q6 movement required");
        return ESP_OK;
    }

    ESP_LOGD(TAG, "starting local q6 move, steps=%u", static_cast<unsigned>(abs_steps));

    for (uint32_t step_index = 0; step_index < abs_steps; ++step_index) {
        if (robo_6dof::robot_state::is_estop() ||
            robo_6dof::robot_state::mode() != robo_6dof::robot_state::RobotMode::Moving) {
            robo_6dof::stepper_a4988::emergency_stop();
            ESP_LOGW(TAG, "q6 move interrupted at step %u/%u",
                     static_cast<unsigned>(step_index),
                     static_cast<unsigned>(abs_steps));
            return ESP_ERR_INVALID_STATE;
        }

        ESP_RETURN_ON_ERROR(robo_6dof::stepper_a4988::pulse(axis), TAG, "failed to pulse q6");
        esp_rom_delay_us(kStepIntervalUs);
    }

    ESP_LOGD(TAG, "local q6 move complete");
    return ESP_OK;
}

std::array<float, robo_6dof::board_config::kMegaJointCount> mega_targets_from(
    const std::array<float, robo_6dof::board_config::kJointCount>& targets_deg)
{
    std::array<float, robo_6dof::board_config::kMegaJointCount> mega_targets = {};
    for (std::size_t axis = 0; axis < mega_targets.size(); ++axis) {
        mega_targets[axis] = targets_deg[axis];
    }
    return mega_targets;
}
} // namespace

namespace robo_6dof {
namespace motion_control {

esp_err_t init()
{
    ESP_LOGD(TAG, "module ready");
    return ESP_OK;
}

esp_err_t move_to_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg)
{
    if (!targets_in_limits(targets_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    const auto current = robot_state::snapshot();

    const esp_err_t prepare_err = mega_bridge::prepare_move(mega_targets_from(targets_deg));
    if (prepare_err != ESP_OK) {
        return prepare_err;
    }

    if (robot_state::begin_motion() != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t start_mega_err = mega_bridge::start_prepared_move();
    if (start_mega_err != ESP_OK) {
        robot_state::stop();
        return start_mega_err;
    }

    const esp_err_t move_err = execute_blocking_move(current.joints_deg, targets_deg);
    if (move_err != ESP_OK) {
        mega_bridge::stop();
        robot_state::stop();
        return move_err;
    }

    const esp_err_t mega_done_err = mega_bridge::wait_for_move_done();
    if (mega_done_err != ESP_OK) {
        robot_state::stop();
        return mega_done_err;
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
