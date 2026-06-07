#include "robot_state.hpp"

#include <algorithm>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr char TAG[] = "robot_state";

struct StateData {
    std::array<float, robo_6dof::board_config::kJointCount> joints_deg;
    int gripper_percent;
    bool armed;
    bool estop;
};

SemaphoreHandle_t g_lock = nullptr;
StateData g_state = {};

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
        load_home_positions();
        g_state.gripper_percent = 0;
        g_state.armed = false;
        g_state.estop = false;
    }

    ESP_LOGI(TAG, "module ready with manual-home estimated position");
    return ESP_OK;
}

esp_err_t start()
{
    StateLock lock;

    if (g_state.estop) {
        return ESP_ERR_INVALID_STATE;
    }

    g_state.armed = true;
    return ESP_OK;
}

esp_err_t stop()
{
    StateLock lock;
    g_state.armed = false;
    return ESP_OK;
}

esp_err_t emergency_stop()
{
    StateLock lock;
    g_state.armed = false;
    g_state.estop = true;
    return ESP_OK;
}

bool is_armed()
{
    StateLock lock;
    return g_state.armed;
}

bool is_estop()
{
    StateLock lock;
    return g_state.estop;
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
    g_state.joints_deg = targets_deg;
    return ESP_OK;
}

esp_err_t set_home_position()
{
    StateLock lock;
    load_home_positions();
    return ESP_OK;
}

esp_err_t set_gripper_percent(int percent)
{
    StateLock lock;
    g_state.gripper_percent = clamp_gripper_percent(percent);
    return ESP_OK;
}

Snapshot snapshot()
{
    StateLock lock;
    return {
        g_state.joints_deg,
        g_state.gripper_percent,
        g_state.armed,
        g_state.estop,
    };
}

} // namespace robot_state
} // namespace robo_6dof
