#include "board_config.hpp"

#include <cmath>
#include <cstdint>

#include "esp_log.h"

namespace {
constexpr char TAG[] = "board_config";

constexpr robo_6dof::board_config::HardwarePins kHardwarePins = {
    GPIO_NUM_34,
    GPIO_NUM_35,
    GPIO_NUM_2,
    GPIO_NUM_4,
};

constexpr robo_6dof::board_config::JointConfig kJointConfigs[] = {
    {
        "Base",
        GPIO_NUM_13,
        GPIO_NUM_14,
        -180.0F,
        180.0F,
        0.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
    {
        "Ombro",
        GPIO_NUM_16,
        GPIO_NUM_17,
        -90.0F,
        90.0F,
        0.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
    {
        "Cotovelo",
        GPIO_NUM_18,
        GPIO_NUM_19,
        -180.0F,
        180.0F,
        90.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
    {
        "Punho 1",
        GPIO_NUM_23,
        GPIO_NUM_25,
        -180.0F,
        180.0F,
        0.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
    {
        "Punho 2",
        GPIO_NUM_26,
        GPIO_NUM_27,
        -180.0F,
        180.0F,
        0.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
    {
        "Garra Rot.",
        GPIO_NUM_32,
        GPIO_NUM_33,
        -180.0F,
        180.0F,
        0.0F,
        200,
        16,
        1.0F,
        false,
        30.0F,
        60.0F,
    },
};

static_assert(
    sizeof(kJointConfigs) / sizeof(kJointConfigs[0]) == robo_6dof::board_config::kJointCount,
    "Joint table must match kJointCount");

uint64_t gpio_bit(gpio_num_t gpio)
{
    return 1ULL << static_cast<unsigned>(gpio);
}

esp_err_t configure_outputs()
{
    uint64_t output_mask = gpio_bit(kHardwarePins.gripper_servo_gpio);

    for (const auto& joint : kJointConfigs) {
        output_mask |= gpio_bit(joint.step_gpio);
        output_mask |= gpio_bit(joint.dir_gpio);
    }

    gpio_config_t output_config = {};
    output_config.pin_bit_mask = output_mask;
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_DISABLE;
    output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&output_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure output GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    for (const auto& joint : kJointConfigs) {
        gpio_set_level(joint.step_gpio, 0);
        gpio_set_level(joint.dir_gpio, 0);
    }
    gpio_set_level(kHardwarePins.gripper_servo_gpio, 0);

    return ESP_OK;
}

esp_err_t configure_inputs()
{
    const uint64_t input_mask =
        gpio_bit(kHardwarePins.start_button_gpio) |
        gpio_bit(kHardwarePins.stop_estop_button_gpio) |
        gpio_bit(kHardwarePins.limit_switch_gpio);

    gpio_config_t input_config = {};
    input_config.pin_bit_mask = input_mask;
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pull_up_en = GPIO_PULLUP_DISABLE;
    input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&input_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure input GPIOs: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

bool validate_home_positions()
{
    for (std::size_t axis = 0; axis < robo_6dof::board_config::kJointCount; ++axis) {
        const auto& joint = kJointConfigs[axis];
        if (!robo_6dof::board_config::validate_joint_target_deg(axis, joint.home_deg)) {
            ESP_LOGE(TAG, "invalid home for q%u (%s): %.1f deg",
                     static_cast<unsigned>(axis + 1),
                     joint.name,
                     static_cast<double>(joint.home_deg));
            return false;
        }
    }

    return true;
}

void log_loaded_config()
{
    ESP_LOGI(TAG,
             "pins: START=GPIO%d STOP/ESTOP=GPIO%d FIM_DE_CURSO=GPIO%d SERVO_GARRA=GPIO%d",
             static_cast<int>(kHardwarePins.start_button_gpio),
             static_cast<int>(kHardwarePins.stop_estop_button_gpio),
             static_cast<int>(kHardwarePins.limit_switch_gpio),
             static_cast<int>(kHardwarePins.gripper_servo_gpio));

    for (std::size_t axis = 0; axis < robo_6dof::board_config::kJointCount; ++axis) {
        const auto& joint = kJointConfigs[axis];
        ESP_LOGI(TAG,
                 "q%u %s: STEP=GPIO%d DIR=GPIO%d home=%.1f min=%.1f max=%.1f "
                 "steps/rev=%d microstep=%d gear=%.2f invert=%s max_speed=%.1f max_accel=%.1f",
                 static_cast<unsigned>(axis + 1),
                 joint.name,
                 static_cast<int>(joint.step_gpio),
                 static_cast<int>(joint.dir_gpio),
                 static_cast<double>(joint.home_deg),
                 static_cast<double>(joint.min_deg),
                 static_cast<double>(joint.max_deg),
                 joint.steps_per_rev,
                 joint.microstep,
                 static_cast<double>(joint.gear_ratio),
                 joint.invert_dir ? "yes" : "no",
                 static_cast<double>(joint.max_speed_dps),
                 static_cast<double>(joint.max_accel_dps2));
    }
}
} // namespace

namespace robo_6dof {
namespace board_config {

esp_err_t init()
{
    if (!validate_home_positions()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = configure_outputs();
    if (err != ESP_OK) {
        return err;
    }

    err = configure_inputs();
    if (err != ESP_OK) {
        return err;
    }

    log_loaded_config();
    ESP_LOGI(TAG, "module ready");
    return ESP_OK;
}

const HardwarePins& hardware_pins()
{
    return kHardwarePins;
}

const JointConfig* joint_config(std::size_t axis)
{
    if (axis >= kJointCount) {
        return nullptr;
    }

    return &kJointConfigs[axis];
}

const JointConfig* joint_configs()
{
    return kJointConfigs;
}

std::size_t joint_count()
{
    return kJointCount;
}

bool validate_joint_target_deg(std::size_t axis, float target_deg)
{
    const JointConfig* joint = joint_config(axis);
    if (joint == nullptr || !std::isfinite(target_deg)) {
        return false;
    }

    return target_deg >= joint->min_deg && target_deg <= joint->max_deg;
}

bool validate_joint_targets_deg(
    const float* targets_deg,
    std::size_t count,
    std::size_t* first_invalid_axis)
{
    if (targets_deg == nullptr || count != kJointCount) {
        if (first_invalid_axis != nullptr) {
            *first_invalid_axis = 0;
        }
        return false;
    }

    for (std::size_t axis = 0; axis < kJointCount; ++axis) {
        if (!validate_joint_target_deg(axis, targets_deg[axis])) {
            if (first_invalid_axis != nullptr) {
                *first_invalid_axis = axis;
            }
            return false;
        }
    }

    return true;
}

} // namespace board_config
} // namespace robo_6dof
