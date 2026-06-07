#include "stepper_a4988.hpp"

#include "board_config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

namespace {
constexpr char TAG[] = "stepper_a4988";
constexpr uint32_t kStepPulseHighUs = 5;

const robo_6dof::board_config::JointConfig* checked_joint(std::size_t axis)
{
    const auto* joint = robo_6dof::board_config::joint_config(axis);
    if (joint == nullptr) {
        ESP_LOGE(TAG, "invalid axis %u", static_cast<unsigned>(axis));
    }
    return joint;
}
} // namespace

namespace robo_6dof {
namespace stepper_a4988 {

esp_err_t init()
{
    for (std::size_t axis = 0; axis < board_config::joint_count(); ++axis) {
        const auto* joint = board_config::joint_config(axis);
        if (joint == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        gpio_set_level(joint->step_gpio, 0);
        gpio_set_level(joint->dir_gpio, 0);
    }

    ESP_LOGD(TAG, "module ready with STEP/DIR outputs idle low");
    return ESP_OK;
}

esp_err_t set_direction(std::size_t axis, bool positive_direction)
{
    const auto* joint = checked_joint(axis);
    if (joint == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool logical_level = joint->invert_dir ? !positive_direction : positive_direction;
    gpio_set_level(joint->dir_gpio, logical_level ? 1 : 0);
    return ESP_OK;
}

esp_err_t pulse(std::size_t axis)
{
    const auto* joint = checked_joint(axis);
    if (joint == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(joint->step_gpio, 1);
    esp_rom_delay_us(kStepPulseHighUs);
    gpio_set_level(joint->step_gpio, 0);
    return ESP_OK;
}

esp_err_t disable_all()
{
    for (std::size_t axis = 0; axis < board_config::joint_count(); ++axis) {
        const auto* joint = board_config::joint_config(axis);
        if (joint == nullptr) {
            return ESP_ERR_INVALID_ARG;
        }

        gpio_set_level(joint->step_gpio, 0);
    }

    return ESP_OK;
}

esp_err_t emergency_stop()
{
    return disable_all();
}

} // namespace stepper_a4988
} // namespace robo_6dof
