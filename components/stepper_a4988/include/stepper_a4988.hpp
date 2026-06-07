#pragma once

#include <cstddef>

#include "esp_err.h"

namespace robo_6dof {
namespace stepper_a4988 {

esp_err_t init();
esp_err_t set_direction(std::size_t axis, bool positive_direction);
esp_err_t pulse(std::size_t axis);
esp_err_t disable_all();
esp_err_t emergency_stop();

} // namespace stepper_a4988
} // namespace robo_6dof
