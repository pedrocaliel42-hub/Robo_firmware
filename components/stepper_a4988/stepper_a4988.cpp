#include "stepper_a4988.hpp"

#include "esp_log.h"

namespace {
constexpr char TAG[] = "stepper_a4988";
} // namespace

namespace robo_6dof {
namespace stepper_a4988 {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

} // namespace stepper_a4988
} // namespace robo_6dof
