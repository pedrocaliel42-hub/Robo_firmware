#include "gripper_servo.hpp"

#include "esp_log.h"

namespace {
constexpr char TAG[] = "gripper_servo";
} // namespace

namespace robo_6dof {
namespace gripper_servo {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

} // namespace gripper_servo
} // namespace robo_6dof
