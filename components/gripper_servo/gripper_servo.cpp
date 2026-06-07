#include "gripper_servo.hpp"

#include <algorithm>
#include <cstdint>

#include "board_config.hpp"
#include "driver/ledc.h"
#include "esp_log.h"

namespace {
constexpr char TAG[] = "gripper_servo";
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kLedcDutyResolution = LEDC_TIMER_16_BIT;
constexpr uint32_t kPwmFrequencyHz = 50;
constexpr uint32_t kPeriodUs = 1000000 / kPwmFrequencyHz;
constexpr uint32_t kClosedPulseUs = 1000;
constexpr uint32_t kOpenPulseUs = 2000;
constexpr uint32_t kMaxDuty = (1UL << 16) - 1UL;

uint32_t pulse_width_us_from_percent(int percent)
{
    const int clamped = std::clamp(percent, 0, 100);
    return kClosedPulseUs +
        ((kOpenPulseUs - kClosedPulseUs) * static_cast<uint32_t>(clamped)) / 100U;
}

uint32_t duty_from_pulse_width_us(uint32_t pulse_width_us)
{
    return (pulse_width_us * kMaxDuty) / kPeriodUs;
}
} // namespace

namespace robo_6dof {
namespace gripper_servo {

esp_err_t init()
{
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kLedcMode;
    timer_config.duty_resolution = kLedcDutyResolution;
    timer_config.timer_num = kLedcTimer;
    timer_config.freq_hz = kPwmFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure LEDC timer: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = board_config::hardware_pins().gripper_servo_gpio;
    channel_config.speed_mode = kLedcMode;
    channel_config.channel = kLedcChannel;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = kLedcTimer;
    channel_config.duty = duty_from_pulse_width_us(pulse_width_us_from_percent(0));
    channel_config.hpoint = 0;

    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure LEDC channel: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG,
             "module ready on GPIO%d, %u Hz, %u-%u us",
             static_cast<int>(board_config::hardware_pins().gripper_servo_gpio),
             static_cast<unsigned>(kPwmFrequencyHz),
             static_cast<unsigned>(kClosedPulseUs),
             static_cast<unsigned>(kOpenPulseUs));
    return ESP_OK;
}

esp_err_t set_percent(int percent)
{
    const int clamped = std::clamp(percent, 0, 100);
    const uint32_t pulse_width_us = pulse_width_us_from_percent(clamped);
    const uint32_t duty = duty_from_pulse_width_us(pulse_width_us);

    esp_err_t err = ledc_set_duty(kLedcMode, kLedcChannel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set gripper duty: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_update_duty(kLedcMode, kLedcChannel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to update gripper duty: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG,
             "gripper target %d%% -> %u us",
             clamped,
             static_cast<unsigned>(pulse_width_us));
    return ESP_OK;
}

} // namespace gripper_servo
} // namespace robo_6dof
