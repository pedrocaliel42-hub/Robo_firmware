#include "robot_state.hpp"

#include "esp_log.h"

namespace {
constexpr char TAG[] = "robot_state";
} // namespace

namespace robo_6dof {
namespace robot_state {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

} // namespace robot_state
} // namespace robo_6dof
