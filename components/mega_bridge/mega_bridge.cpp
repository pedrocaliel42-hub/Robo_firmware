#include "mega_bridge.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
constexpr char TAG[] = "mega_bridge";
constexpr uart_port_t kMegaUart = UART_NUM_2;
constexpr int kBaudRate = 115200;
constexpr int kRxBufferSize = 512;
constexpr int kTxBufferSize = 512;
constexpr uint32_t kResponseTimeoutMs = 2500;
constexpr std::size_t kLineBufferLength = 128;

SemaphoreHandle_t g_lock = nullptr;
uint32_t g_sequence = 0;

class BridgeLock {
public:
    BridgeLock()
    {
        if (g_lock != nullptr) {
            xSemaphoreTake(g_lock, portMAX_DELAY);
            locked_ = true;
        }
    }

    ~BridgeLock()
    {
        if (locked_) {
            xSemaphoreGive(g_lock);
        }
    }

    BridgeLock(const BridgeLock&) = delete;
    BridgeLock& operator=(const BridgeLock&) = delete;

private:
    bool locked_ = false;
};

void trim_line(char* line)
{
    std::size_t length = std::strlen(line);
    while (length > 0 &&
           std::isspace(static_cast<unsigned char>(line[length - 1])) != 0) {
        line[--length] = '\0';
    }
}

esp_err_t write_command(const char* command)
{
    uart_flush_input(kMegaUart);

    const int written = uart_write_bytes(kMegaUart, command, std::strlen(command));
    if (written < 0 || static_cast<std::size_t>(written) != std::strlen(command)) {
        ESP_LOGE(TAG, "failed to write command to Mega");
        return ESP_FAIL;
    }

    const int newline_written = uart_write_bytes(kMegaUart, "\n", 1);
    if (newline_written != 1) {
        ESP_LOGE(TAG, "failed to write command terminator to Mega");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "TX Mega: %s", command);
    return ESP_OK;
}

esp_err_t read_line(char* line, std::size_t line_length, uint32_t timeout_ms)
{
    if (line == nullptr || line_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    std::size_t index = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        uint8_t byte = 0;
        const int read = uart_read_bytes(kMegaUart, &byte, 1, pdMS_TO_TICKS(20));
        if (read == 0) {
            continue;
        }
        if (read < 0) {
            return ESP_FAIL;
        }

        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            if (index == 0) {
                continue;
            }

            line[index] = '\0';
            trim_line(line);
            ESP_LOGD(TAG, "RX Mega: %s", line);
            return ESP_OK;
        }

        if (!std::isprint(static_cast<unsigned char>(byte))) {
            continue;
        }

        if (index >= line_length - 1) {
            line[0] = '\0';
            return ESP_ERR_INVALID_SIZE;
        }

        line[index++] = static_cast<char>(byte);
    }

    line[0] = '\0';
    ESP_LOGW(TAG, "Mega response timeout");
    return ESP_ERR_TIMEOUT;
}

bool response_is_error(const char* line)
{
    return std::strncmp(line, "MERR_", 5) == 0;
}

esp_err_t command_expect_exact(const char* command, const char* expected)
{
    ESP_RETURN_ON_ERROR(write_command(command), TAG, "write failed");

    char line[kLineBufferLength] = {};
    ESP_RETURN_ON_ERROR(read_line(line, sizeof(line), kResponseTimeoutMs), TAG, "read failed");

    if (std::strcmp(line, expected) == 0) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "unexpected Mega response: expected '%s', got '%s'", expected, line);
    return response_is_error(line) ? ESP_ERR_INVALID_ARG : ESP_FAIL;
}

uint32_t next_sequence()
{
    ++g_sequence;
    if (g_sequence == 0) {
        ++g_sequence;
    }
    return g_sequence;
}
} // namespace

namespace robo_6dof {
namespace mega_bridge {

esp_err_t init()
{
    if (g_lock == nullptr) {
        g_lock = xSemaphoreCreateMutex();
        if (g_lock == nullptr) {
            ESP_LOGE(TAG, "failed to create bridge mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    const auto& pins = board_config::hardware_pins();

    uart_config_t uart_config = {};
    uart_config.baud_rate = kBaudRate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_driver_install(
                            kMegaUart,
                            kRxBufferSize,
                            kTxBufferSize,
                            0,
                            nullptr,
                            0),
                        TAG,
                        "failed to install UART driver");

    ESP_RETURN_ON_ERROR(uart_param_config(kMegaUart, &uart_config), TAG, "failed to config UART");
    ESP_RETURN_ON_ERROR(uart_set_pin(
                            kMegaUart,
                            pins.mega_uart_tx_gpio,
                            pins.mega_uart_rx_gpio,
                            UART_PIN_NO_CHANGE,
                            UART_PIN_NO_CHANGE),
                        TAG,
                        "failed to set UART pins");

    ESP_LOGD(TAG,
             "module ready on UART2 RX=GPIO%d TX=GPIO%d baud=%d",
             static_cast<int>(pins.mega_uart_rx_gpio),
             static_cast<int>(pins.mega_uart_tx_gpio),
             kBaudRate);
    return ESP_OK;
}

esp_err_t ping()
{
    BridgeLock lock;
    return command_expect_exact("MPING", "MOK_PONG");
}

esp_err_t reference_home()
{
    BridgeLock lock;
    return command_expect_exact("MREF", "MOK_REF_HOME");
}

esp_err_t prepare_move(const std::array<float, board_config::kMegaJointCount>& targets_deg)
{
    BridgeLock lock;
    const uint32_t sequence = next_sequence();

    char command[kLineBufferLength] = {};
    std::snprintf(command,
                  sizeof(command),
                  "MPREP,%lu,%.3f,%.3f,%.3f,%.3f,%.3f",
                  static_cast<unsigned long>(sequence),
                  static_cast<double>(targets_deg[0]),
                  static_cast<double>(targets_deg[1]),
                  static_cast<double>(targets_deg[2]),
                  static_cast<double>(targets_deg[3]),
                  static_cast<double>(targets_deg[4]));

    ESP_RETURN_ON_ERROR(write_command(command), TAG, "write MPREP failed");

    char expected[kLineBufferLength] = {};
    std::snprintf(expected, sizeof(expected), "MREADY,%lu", static_cast<unsigned long>(sequence));

    char line[kLineBufferLength] = {};
    ESP_RETURN_ON_ERROR(read_line(line, sizeof(line), kResponseTimeoutMs), TAG, "read MPREP failed");

    if (std::strcmp(line, expected) != 0) {
        ESP_LOGW(TAG, "unexpected MPREP response: expected '%s', got '%s'", expected, line);
        return response_is_error(line) ? ESP_ERR_INVALID_ARG : ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t start_prepared_move()
{
    BridgeLock lock;

    char command[kLineBufferLength] = {};
    std::snprintf(command, sizeof(command), "MGO,%lu", static_cast<unsigned long>(g_sequence));
    return write_command(command);
}

esp_err_t wait_for_move_done(uint32_t timeout_ms)
{
    BridgeLock lock;

    // timeout_ms == 0 usa o timeout padrao; movimentos longos passam um valor maior.
    const uint32_t effective_timeout =
        (timeout_ms < kResponseTimeoutMs) ? kResponseTimeoutMs : timeout_ms;

    char expected[kLineBufferLength] = {};
    std::snprintf(expected, sizeof(expected), "MDONE,%lu", static_cast<unsigned long>(g_sequence));

    char line[kLineBufferLength] = {};
    ESP_RETURN_ON_ERROR(read_line(line, sizeof(line), effective_timeout), TAG, "read MDONE failed");

    if (std::strcmp(line, expected) == 0) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "unexpected MGO response: expected '%s', got '%s'", expected, line);
    return response_is_error(line) ? ESP_ERR_INVALID_ARG : ESP_FAIL;
}

esp_err_t stop()
{
    BridgeLock lock;

    char command[kLineBufferLength] = {};
    char expected[kLineBufferLength] = {};
    std::snprintf(command, sizeof(command), "MSTOP,%lu", static_cast<unsigned long>(g_sequence));
    std::snprintf(expected, sizeof(expected), "MSTOPPED,%lu", static_cast<unsigned long>(g_sequence));
    return command_expect_exact(command, expected);
}

esp_err_t emergency_stop()
{
    BridgeLock lock;
    return command_expect_exact("MESTOP", "MOK_ESTOP");
}

esp_err_t set_gripper_percent(int percent)
{
    BridgeLock lock;
    const int clamped = (percent < 0) ? 0 : (percent > 100) ? 100 : percent;
    char command[kLineBufferLength] = {};
    std::snprintf(command, sizeof(command), "MGRP,%d", clamped);
    return command_expect_exact(command, "MOK_GRIP");
}

esp_err_t jog_relative(
    const std::array<float, board_config::kMegaJointCount>& deltas_deg,
    uint32_t timeout_ms)
{
    BridgeLock lock;

    const uint32_t effective_timeout =
        (timeout_ms < kResponseTimeoutMs) ? kResponseTimeoutMs : timeout_ms;

    char command[kLineBufferLength] = {};
    std::snprintf(command,
                  sizeof(command),
                  "MJOG,%.3f,%.3f,%.3f,%.3f,%.3f",
                  static_cast<double>(deltas_deg[0]),
                  static_cast<double>(deltas_deg[1]),
                  static_cast<double>(deltas_deg[2]),
                  static_cast<double>(deltas_deg[3]),
                  static_cast<double>(deltas_deg[4]));

    ESP_RETURN_ON_ERROR(write_command(command), TAG, "write MJOG failed");

    char line[kLineBufferLength] = {};
    ESP_RETURN_ON_ERROR(read_line(line, sizeof(line), effective_timeout), TAG, "read MJOG failed");

    if (std::strcmp(line, "MJOGDONE") == 0) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "unexpected MJOG response: expected 'MJOGDONE', got '%s'", line);
    return response_is_error(line) ? ESP_ERR_INVALID_ARG : ESP_FAIL;
}

} // namespace mega_bridge
} // namespace robo_6dof
