#include "robot_state.hpp"

#include <algorithm>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {
constexpr char TAG[] = "robot_state";
constexpr uint32_t kSafetyPollPeriodMs = 20;
constexpr uint32_t kDebouncePeriodMs = 60;
constexpr int kInputActiveLevel = 1;

struct StateData {
    std::array<float, robo_6dof::board_config::kJointCount> joints_deg;
    int gripper_percent;
    robo_6dof::robot_state::RobotMode mode;
};

SemaphoreHandle_t g_lock = nullptr;
StateData g_state = {};
bool g_safety_task_started = false;

struct DebouncedInput {
    int stable_level = 0;
    int last_level = 0;
    TickType_t changed_at = 0;
};

class StateLock {
public:
    StateLock()
    {
        if (g_lock != nullptr) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            locked_ = true;
        }
    }

    ~StateLock()
    {
        if (locked_) {
            xSemaphoreGive(g_lock);
        }
    }

    StateLock(const StateLock&) = delete;
    StateLock& operator=(const StateLock&) = delete;

private:
    bool locked_ = false;
};

void load_home_positions()
{
    for (std::size_t axis = 0; axis < robo_6dof::board_config::kJointCount; ++axis) {
        const auto* joint = robo_6dof::board_config::joint_config(axis);
        g_state.joints_deg[axis] = joint != nullptr ? joint->home_deg : 0.0F;
    }
}

int clamp_gripper_percent(int percent)
{
    return std::clamp(percent, 0, 100);
}

const char* mode_name_unlocked(robo_6dof::robot_state::RobotMode mode)
{
    using robo_6dof::robot_state::RobotMode;

    switch (mode) {
    case RobotMode::Boot:
        return "BOOT";
    case RobotMode::Idle:
        return "IDLE";
    case RobotMode::Armed:
        return "ARMED";
    case RobotMode::Moving:
        return "MOVING";
    case RobotMode::Stopped:
        return "STOPPED";
    case RobotMode::Estop:
        return "ESTOP";
    case RobotMode::Fault:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

void transition_unlocked(robo_6dof::robot_state::RobotMode next)
{
    if (g_state.mode == next) {
        return;
    }

    ESP_LOGD(TAG, "state %s -> %s", mode_name_unlocked(g_state.mode), mode_name_unlocked(next));
    g_state.mode = next;
}

bool can_accept_motion_unlocked()
{
    return g_state.mode == robo_6dof::robot_state::RobotMode::Armed;
}

bool update_debounced_input(DebouncedInput& input, int raw_level, TickType_t now)
{
    if (raw_level != input.last_level) {
        input.last_level = raw_level;
        input.changed_at = now;
        return false;
    }

    if (raw_level != input.stable_level &&
        (now - input.changed_at) >= pdMS_TO_TICKS(kDebouncePeriodMs)) {
        input.stable_level = raw_level;
        return true;
    }

    return false;
}

void safety_input_task(void*)
{
    const auto& pins = robo_6dof::board_config::hardware_pins();

    DebouncedInput stop_input = {};
    stop_input.stable_level = gpio_get_level(pins.stop_estop_button_gpio);
    stop_input.last_level = stop_input.stable_level;
    stop_input.changed_at = xTaskGetTickCount();

    DebouncedInput limit_input = {};
    limit_input.stable_level = gpio_get_level(pins.limit_switch_gpio);
    limit_input.last_level = limit_input.stable_level;
    limit_input.changed_at = xTaskGetTickCount();

    bool last_limit_active = limit_input.stable_level == kInputActiveLevel;
    ESP_LOGD(TAG, "safety input task ready");

    while (true) {
        const TickType_t now = xTaskGetTickCount();

        if (update_debounced_input(
                stop_input,
                gpio_get_level(pins.stop_estop_button_gpio),
                now) &&
            stop_input.stable_level == kInputActiveLevel) {
            ESP_LOGW(TAG, "physical STOP/ESTOP input active");
            robo_6dof::robot_state::emergency_stop();
        }

        if (update_debounced_input(
                limit_input,
                gpio_get_level(pins.limit_switch_gpio),
                now)) {
            const bool limit_active = limit_input.stable_level == kInputActiveLevel;
            if (limit_active != last_limit_active) {
                ESP_LOGW(TAG, "limit switch %s", limit_active ? "active" : "inactive");
                last_limit_active = limit_active;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(kSafetyPollPeriodMs));
    }
}
} // namespace

namespace robo_6dof {
namespace robot_state {

esp_err_t init()
{
    if (g_lock == nullptr) {
        g_lock = xSemaphoreCreateMutex();
        if (g_lock == nullptr) {
            ESP_LOGE(TAG, "failed to create state mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    {
        StateLock lock;
        g_state.mode = RobotMode::Boot;
        load_home_positions();
        g_state.gripper_percent = 0;
        transition_unlocked(RobotMode::Idle);
    }

    if (!g_safety_task_started) {
        BaseType_t created = xTaskCreate(
            safety_input_task,
            "safety_inputs",
            3072,
            nullptr,
            6,
            nullptr);
        if (created != pdPASS) {
            ESP_LOGE(TAG, "failed to create safety input task");
            return ESP_ERR_NO_MEM;
        }
        g_safety_task_started = true;
    }

    ESP_LOGD(TAG, "module ready with manual-home estimated position");
    return ESP_OK;
}

esp_err_t start()
{
    StateLock lock;

    if (g_state.mode == RobotMode::Estop) {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_state.mode == RobotMode::Fault) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t stop()
{
    StateLock lock;
    if (g_state.mode == RobotMode::Estop) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Stopped);
    return ESP_OK;
}

esp_err_t emergency_stop()
{
    StateLock lock;
    transition_unlocked(RobotMode::Estop);
    return ESP_OK;
}

bool is_armed()
{
    StateLock lock;
    return g_state.mode == RobotMode::Armed;
}

bool is_estop()
{
    StateLock lock;
    return g_state.mode == RobotMode::Estop;
}

bool can_accept_motion()
{
    StateLock lock;
    return can_accept_motion_unlocked();
}

RobotMode mode()
{
    StateLock lock;
    return g_state.mode;
}

const char* mode_name(RobotMode mode)
{
    return mode_name_unlocked(mode);
}

esp_err_t begin_motion()
{
    StateLock lock;

    if (!can_accept_motion_unlocked()) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Moving);
    return ESP_OK;
}

esp_err_t finish_jog()
{
    StateLock lock;

    if (g_state.mode != RobotMode::Moving) {
        return ESP_ERR_INVALID_STATE;
    }

    // Jog relativo não memoriza posição: apenas volta para Armed.
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t finish_joint_motion(const std::array<float, board_config::kJointCount>& reached_deg)
{
    std::size_t first_invalid_axis = 0;
    if (!board_config::validate_joint_targets_deg(
            reached_deg.data(),
            reached_deg.size(),
            &first_invalid_axis)) {
        ESP_LOGW(TAG, "reached target out of limits on q%u", static_cast<unsigned>(first_invalid_axis + 1));
        return ESP_ERR_INVALID_ARG;
    }

    StateLock lock;

    if (g_state.mode != RobotMode::Moving) {
        return ESP_ERR_INVALID_STATE;
    }

    g_state.joints_deg = reached_deg;
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t finish_home_motion()
{
    StateLock lock;

    if (g_state.mode != RobotMode::Moving) {
        return ESP_ERR_INVALID_STATE;
    }

    load_home_positions();
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t set_joint_targets_deg(const std::array<float, board_config::kJointCount>& targets_deg)
{
    std::size_t first_invalid_axis = 0;
    if (!board_config::validate_joint_targets_deg(
            targets_deg.data(),
            targets_deg.size(),
            &first_invalid_axis)) {
        ESP_LOGW(TAG, "target out of limits on q%u", static_cast<unsigned>(first_invalid_axis + 1));
        return ESP_ERR_INVALID_ARG;
    }

    StateLock lock;
    if (!can_accept_motion_unlocked()) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Moving);
    g_state.joints_deg = targets_deg;
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t set_home_position()
{
    StateLock lock;
    if (!can_accept_motion_unlocked()) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Moving);
    load_home_positions();
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

esp_err_t set_gripper_percent(int percent)
{
    StateLock lock;
    if (!can_accept_motion_unlocked()) {
        return ESP_ERR_INVALID_STATE;
    }

    transition_unlocked(RobotMode::Moving);
    g_state.gripper_percent = clamp_gripper_percent(percent);
    transition_unlocked(RobotMode::Armed);
    return ESP_OK;
}

Snapshot snapshot()
{
    StateLock lock;
    return {
        g_state.joints_deg,
        g_state.gripper_percent,
        g_state.mode,
    };
}

} // namespace robot_state
} // namespace robo_6dof
