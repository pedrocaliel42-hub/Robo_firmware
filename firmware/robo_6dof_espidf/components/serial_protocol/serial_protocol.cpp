#include "serial_protocol.hpp"

#include <cstdio>

#include "esp_log.h"

namespace {
constexpr char TAG[] = "serial_protocol";
} // namespace

namespace robo_6dof {
namespace serial_protocol {

esp_err_t init()
{
    ESP_LOGI(TAG, "module ready on default console UART");
    return ESP_OK;
}

esp_err_t send_boot_message()
{
    const int written = std::printf("OK_INIT_ROBOT\n");
    std::fflush(stdout);

    return written > 0 ? ESP_OK : ESP_FAIL;
}

} // namespace serial_protocol
} // namespace robo_6dof
