#pragma once

#include "esp_err.h"

namespace robo_6dof {
namespace serial_protocol {

esp_err_t init();
esp_err_t start();
esp_err_t send_boot_message();

} // namespace serial_protocol
} // namespace robo_6dof
