#include "motion_control.hpp"

#include "esp_log.h"

namespace {
constexpr char TAG[] = "motion_control";
} // namespace

namespace robo_6dof {
namespace motion_control {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

} // namespace motion_control
} // namespace robo_6dof
