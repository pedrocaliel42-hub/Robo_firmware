#include "sensors.hpp"

#include "esp_log.h"

namespace {
constexpr char TAG[] = "sensors";
} // namespace

namespace robo_6dof {
namespace sensors {

esp_err_t init()
{
    ESP_LOGD(TAG, "module ready");
    return ESP_OK;
}

} // namespace sensors
} // namespace robo_6dof
