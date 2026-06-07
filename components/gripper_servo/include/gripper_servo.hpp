#pragma once

#include "esp_err.h"

namespace robo_6dof {
namespace gripper_servo {

esp_err_t init();
esp_err_t set_percent(int percent);

} // namespace gripper_servo
} // namespace robo_6dof
