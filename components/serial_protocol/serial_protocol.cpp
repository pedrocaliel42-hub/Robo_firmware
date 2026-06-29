#include "serial_protocol.hpp"

#include <array>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "board_config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mega_bridge.hpp"
#include "motion_control.hpp"
#include "robot_state.hpp"

namespace {
constexpr char TAG[] = "serial_protocol";

constexpr std::size_t kLineBufferLength = 128;
constexpr std::size_t kMaxTokens = 10;
constexpr uint32_t kPosPeriodMs = 100;

SemaphoreHandle_t g_tx_lock = nullptr;
SemaphoreHandle_t g_pos_stream_lock = nullptr;
bool g_started = false;
bool g_pos_stream_enabled = false;

class TxLock {
public:
    TxLock()
    {
        if (g_tx_lock != nullptr) {
            xSemaphoreTake(g_tx_lock, portMAX_DELAY);
            locked_ = true;
        }
    }

    ~TxLock()
    {
        if (locked_) {
            xSemaphoreGive(g_tx_lock);
        }
    }

    TxLock(const TxLock&) = delete;
    TxLock& operator=(const TxLock&) = delete;

private:
    bool locked_ = false;
};

class PosStreamLock {
public:
    PosStreamLock()
    {
        if (g_pos_stream_lock != nullptr) {
            xSemaphoreTake(g_pos_stream_lock, portMAX_DELAY);
            locked_ = true;
        }
    }

    ~PosStreamLock()
    {
        if (locked_) {
            xSemaphoreGive(g_pos_stream_lock);
        }
    }

    PosStreamLock(const PosStreamLock&) = delete;
    PosStreamLock& operator=(const PosStreamLock&) = delete;

private:
    bool locked_ = false;
};

void write_line_unlocked(const char* line)
{
    std::printf("%s\n", line);
    std::fflush(stdout);
}

void write_line(const char* line)
{
    TxLock lock;
    write_line_unlocked(line);
}

void set_pos_stream_enabled(bool enabled)
{
    PosStreamLock lock;
    g_pos_stream_enabled = enabled;
}

bool pos_stream_enabled()
{
    PosStreamLock lock;
    return g_pos_stream_enabled;
}

void write_pos_snapshot()
{
    const auto state = robo_6dof::robot_state::snapshot();

    TxLock lock;
    std::printf(
        "POS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,0.0,0.0,0.0\n",
        static_cast<double>(state.joints_deg[0]),
        static_cast<double>(state.joints_deg[1]),
        static_cast<double>(state.joints_deg[2]),
        static_cast<double>(state.joints_deg[3]),
        static_cast<double>(state.joints_deg[4]),
        static_cast<double>(state.joints_deg[5]),
        state.gripper_percent);
    std::fflush(stdout);
}

char* trim_token(char* token)
{
    while (*token != '\0' && std::isspace(static_cast<unsigned char>(*token))) {
        ++token;
    }

    char* end = token + std::strlen(token);
    while (end > token && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
        *end = '\0';
    }

    return token;
}

void trim_line_in_place(char* line)
{
    char* trimmed = trim_token(line);
    if (trimmed != line) {
        std::memmove(line, trimmed, std::strlen(trimmed) + 1);
    }
}

void uppercase_ascii(char* text)
{
    for (; *text != '\0'; ++text) {
        *text = static_cast<char>(std::toupper(static_cast<unsigned char>(*text)));
    }
}

std::size_t split_csv(char* line, std::array<char*, kMaxTokens>& tokens)
{
    std::size_t count = 0;
    char* cursor = line;

    while (true) {
        if (count >= tokens.size()) {
            return tokens.size() + 1;
        }

        tokens[count++] = trim_token(cursor);

        char* comma = std::strchr(cursor, ',');
        if (comma == nullptr) {
            break;
        }

        *comma = '\0';
        tokens[count - 1] = trim_token(cursor);
        cursor = comma + 1;
    }

    return count;
}

bool parse_float(const char* token, float* value)
{
    if (token == nullptr || token[0] == '\0' || value == nullptr) {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const float parsed = std::strtof(token, &end);

    if (errno != 0 || end == token || end == nullptr || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

bool parse_gripper_percent(const char* token, int* percent)
{
    float parsed = 0.0F;
    if (!parse_float(token, &parsed) || percent == nullptr) {
        return false;
    }

    *percent = static_cast<int>(std::lround(parsed));
    return true;
}

bool reject_if_not_ready_for_motion()
{
    if (robo_6dof::robot_state::is_estop()) {
        write_line("ERR_ESTOP");
        return true;
    }

    if (!robo_6dof::robot_state::can_accept_motion()) {
        write_line("ERR_NOT_ARMED");
        return true;
    }

    return false;
}

bool reject_if_unreferenced()
{
    if (robo_6dof::robot_state::reference_state() !=
        robo_6dof::robot_state::ReferenceState::Referenced) {
        write_line("ERR_UNREFERENCED");
        return true;
    }
    return false;
}

void handle_ping(std::size_t token_count)
{
    write_line(token_count == 1 ? "OK_PONG" : "ERR_BAD_FORMAT");
}

void handle_start(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (robo_6dof::robot_state::is_estop()) {
        write_line("ERR_ESTOP");
        return;
    }

    if (robo_6dof::robot_state::start() == ESP_OK) {
        write_line("OK_START");
        set_pos_stream_enabled(true);
    } else {
        write_line("ERR_FAULT");
    }
}

void handle_stop(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    robo_6dof::mega_bridge::stop();

    if (robo_6dof::robot_state::stop() != ESP_OK) {
        write_line("ERR_ESTOP");
        return;
    }

    set_pos_stream_enabled(false);
    write_line("OK_STOP");
}

void handle_estop(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    robo_6dof::robot_state::emergency_stop();
    robo_6dof::mega_bridge::emergency_stop();
    set_pos_stream_enabled(false);
    write_line("OK_ESTOP");
}

void handle_pos_query(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    write_pos_snapshot();
}

void handle_pos_on(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    write_line("OK_POS_ON");
    set_pos_stream_enabled(true);
}

void handle_pos_off(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    set_pos_stream_enabled(false);
    write_line("OK_POS_OFF");
}

void handle_home(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (reject_if_not_ready_for_motion()) {
        return;
    }
    if (reject_if_unreferenced()) {
        return;
    }

    const esp_err_t err = robo_6dof::motion_control::move_home();
    if (err == ESP_ERR_INVALID_ARG) {
        write_line("ERR_LIMIT");
    } else if (err == ESP_ERR_NOT_ALLOWED) {
        write_line("ERR_Q2_LOCKED");
    } else if (err == ESP_ERR_INVALID_STATE) {
        write_line(robo_6dof::robot_state::is_estop() ? "ERR_ESTOP" : "ERR_NOT_ARMED");
    } else if (err == ESP_OK) {
        write_line("OK_MOVE_DONE");
    } else {
        write_line("ERR_FAULT");
    }
}

void handle_ref_home(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }
    if (reject_if_not_ready_for_motion()) {
        return;
    }

    const esp_err_t mega_err = robo_6dof::mega_bridge::reference_home();
    if (mega_err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }
    if (robo_6dof::robot_state::confirm_manual_home() != ESP_OK) {
        write_line("ERR_NOT_ARMED");
        return;
    }
    write_line("OK_REF_HOME");
    write_pos_snapshot();
}

void handle_ref_query(std::size_t token_count)
{
    if (token_count != 1) {
        write_line("ERR_BAD_FORMAT");
        return;
    }
    char line[48] = {};
    std::snprintf(
        line,
        sizeof(line),
        "REF,%s",
        robo_6dof::robot_state::reference_name(robo_6dof::robot_state::reference_state()));
    write_line(line);
}

void handle_q23_power(
    const std::array<char*, kMaxTokens>& tokens,
    std::size_t token_count)
{
    if (token_count != 2) {
        write_line("ERR_BAD_FORMAT");
        return;
    }
    if (reject_if_not_ready_for_motion()) {
        return;
    }

    uppercase_ascii(tokens[1]);
    const bool enable = std::strcmp(tokens[1], "ON") == 0;
    if (!enable && std::strcmp(tokens[1], "OFF") != 0) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (robo_6dof::mega_bridge::set_q23_power(enable) != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }
    write_line(enable ? "OK_Q23_ON" : "OK_Q23_OFF");
}

void handle_q2_lock(
    const std::array<char*, kMaxTokens>& tokens,
    std::size_t token_count)
{
    if (token_count != 2) {
        write_line("ERR_BAD_FORMAT");
        return;
    }
    if (reject_if_not_ready_for_motion()) {
        return;
    }

    uppercase_ascii(tokens[1]);
    const bool locked = std::strcmp(tokens[1], "ON") == 0;
    if (!locked && std::strcmp(tokens[1], "OFF") != 0) {
        write_line("ERR_BAD_FORMAT");
        return;
    }
    if (robo_6dof::mega_bridge::set_q2_locked(locked) != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }
    robo_6dof::motion_control::set_q2_locked(locked);
    write_line(locked ? "OK_Q2_LOCKED" : "OK_Q2_UNLOCKED");
}

void handle_ang(const std::array<char*, kMaxTokens>& tokens, std::size_t token_count)
{
    if (token_count != 8) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (reject_if_not_ready_for_motion()) {
        return;
    }
    if (reject_if_unreferenced()) {
        return;
    }

    std::array<float, robo_6dof::board_config::kJointCount> targets = {};
    for (std::size_t axis = 0; axis < targets.size(); ++axis) {
        if (!parse_float(tokens[axis + 1], &targets[axis])) {
            write_line("ERR_BAD_FORMAT");
            return;
        }
    }

    int gripper_percent = 0;
    if (!parse_gripper_percent(tokens[7], &gripper_percent)) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    const esp_err_t move_err = robo_6dof::motion_control::move_to_targets_deg(targets);
    if (move_err == ESP_ERR_INVALID_ARG) {
        write_line("ERR_LIMIT");
        return;
    }
    if (move_err == ESP_ERR_INVALID_STATE) {
        write_line(robo_6dof::robot_state::is_estop() ? "ERR_ESTOP" : "ERR_NOT_ARMED");
        return;
    }
    if (move_err == ESP_ERR_NOT_ALLOWED) {
        write_line("ERR_Q2_LOCKED");
        return;
    }
    if (move_err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    esp_err_t grip_err = robo_6dof::mega_bridge::set_gripper_percent(gripper_percent);
    if (grip_err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    grip_err = robo_6dof::robot_state::set_gripper_percent(gripper_percent);
    if (grip_err == ESP_ERR_INVALID_STATE) {
        write_line(robo_6dof::robot_state::is_estop() ? "ERR_ESTOP" : "ERR_NOT_ARMED");
        return;
    }
    if (grip_err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    write_line("OK_MOVE_DONE");
}

void handle_jog(const std::array<char*, kMaxTokens>& tokens, std::size_t token_count)
{
    // JOG,d1,d2,d3,d4,d5,d6 — incrementos relativos em graus.
    if (token_count != 7) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (reject_if_not_ready_for_motion()) {
        return;
    }

    std::array<float, robo_6dof::board_config::kJointCount> deltas = {};
    for (std::size_t axis = 0; axis < deltas.size(); ++axis) {
        if (!parse_float(tokens[axis + 1], &deltas[axis])) {
            write_line("ERR_BAD_FORMAT");
            return;
        }
    }

    const esp_err_t err = robo_6dof::motion_control::jog_relative(deltas);
    if (err == ESP_ERR_INVALID_ARG) {
        write_line("ERR_LIMIT");
        return;
    }
    if (err == ESP_ERR_INVALID_STATE) {
        write_line(robo_6dof::robot_state::is_estop() ? "ERR_ESTOP" : "ERR_NOT_ARMED");
        return;
    }
    if (err == ESP_ERR_NOT_ALLOWED) {
        write_line("ERR_Q2_LOCKED");
        return;
    }
    if (err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    write_line("OK_MOVE_DONE");
}

void handle_gripper(const std::array<char*, kMaxTokens>& tokens, std::size_t token_count)
{
    if (token_count != 2) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    if (reject_if_not_ready_for_motion()) {
        return;
    }

    int gripper_percent = 0;
    if (!parse_gripper_percent(tokens[1], &gripper_percent)) {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    esp_err_t err = robo_6dof::mega_bridge::set_gripper_percent(gripper_percent);
    if (err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    err = robo_6dof::robot_state::set_gripper_percent(gripper_percent);
    if (err == ESP_ERR_INVALID_STATE) {
        write_line(robo_6dof::robot_state::is_estop() ? "ERR_ESTOP" : "ERR_NOT_ARMED");
        return;
    }
    if (err != ESP_OK) {
        write_line("ERR_FAULT");
        return;
    }

    write_line("OK_GRIPPER_DONE");
}

void handle_line(char* line)
{
    trim_line_in_place(line);
    if (line[0] == '\0') {
        return;
    }

    std::array<char*, kMaxTokens> tokens = {};
    const std::size_t token_count = split_csv(line, tokens);
    if (token_count > tokens.size() || token_count == 0 || tokens[0][0] == '\0') {
        write_line("ERR_BAD_FORMAT");
        return;
    }

    uppercase_ascii(tokens[0]);

    if (std::strcmp(tokens[0], "PING") == 0) {
        handle_ping(token_count);
    } else if (std::strcmp(tokens[0], "START") == 0) {
        handle_start(token_count);
    } else if (std::strcmp(tokens[0], "STOP") == 0) {
        handle_stop(token_count);
    } else if (std::strcmp(tokens[0], "ESTOP") == 0) {
        handle_estop(token_count);
    } else if (std::strcmp(tokens[0], "HOME") == 0 ||
               std::strcmp(tokens[0], "GO_HOME") == 0) {
        handle_home(token_count);
    } else if (std::strcmp(tokens[0], "REF_HOME") == 0) {
        handle_ref_home(token_count);
    } else if (std::strcmp(tokens[0], "REF?") == 0) {
        handle_ref_query(token_count);
    } else if (std::strcmp(tokens[0], "Q23_POWER") == 0) {
        handle_q23_power(tokens, token_count);
    } else if (std::strcmp(tokens[0], "Q2_LOCK") == 0) {
        handle_q2_lock(tokens, token_count);
    } else if (std::strcmp(tokens[0], "ANG") == 0) {
        handle_ang(tokens, token_count);
    } else if (std::strcmp(tokens[0], "GRP") == 0) {
        handle_gripper(tokens, token_count);
    } else if (std::strcmp(tokens[0], "JOG") == 0) {
        handle_jog(tokens, token_count);
    } else if (std::strcmp(tokens[0], "MOV") == 0) {
        write_line("ERR_UNSUPPORTED_MOV");
    } else if (std::strcmp(tokens[0], "POS?") == 0) {
        handle_pos_query(token_count);
    } else if (std::strcmp(tokens[0], "POSON") == 0) {
        handle_pos_on(token_count);
    } else if (std::strcmp(tokens[0], "POSOFF") == 0) {
        handle_pos_off(token_count);
    } else {
        write_line("ERR_BAD_FORMAT");
    }
}

void serial_rx_task(void*)
{
    char line[kLineBufferLength] = {};
    std::size_t line_length = 0;

    while (true) {
        const int ch = std::getchar();
        if (ch == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (line_length == 0) {
                continue;
            }

            line[line_length] = '\0';
            handle_line(line);
            line_length = 0;
            continue;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (line_length > 0) {
                --line_length;
            }
            continue;
        }

        if (!std::isprint(static_cast<unsigned char>(ch))) {
            continue;
        }

        if (line_length >= sizeof(line) - 1) {
            line_length = 0;
            write_line("ERR_BAD_FORMAT");
            continue;
        }

        line[line_length++] = static_cast<char>(ch);
    }
}

void pos_stream_task(void*)
{
    while (true) {
        if (pos_stream_enabled()) {
            write_pos_snapshot();
        }

        vTaskDelay(pdMS_TO_TICKS(kPosPeriodMs));
    }
}
} // namespace

namespace robo_6dof {
namespace serial_protocol {

esp_err_t init()
{
    if (g_tx_lock == nullptr) {
        g_tx_lock = xSemaphoreCreateMutex();
        if (g_tx_lock == nullptr) {
            ESP_LOGE(TAG, "failed to create TX mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_pos_stream_lock == nullptr) {
        g_pos_stream_lock = xSemaphoreCreateMutex();
        if (g_pos_stream_lock == nullptr) {
            ESP_LOGE(TAG, "failed to create POS stream mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    set_pos_stream_enabled(false);
    ESP_LOGD(TAG, "module ready on default console UART");
    return ESP_OK;
}

esp_err_t start()
{
    if (g_started) {
        return ESP_OK;
    }

    BaseType_t created = xTaskCreate(
        serial_rx_task,
        "serial_rx",
        4096,
        nullptr,
        5,
        nullptr);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "failed to create serial RX task");
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(
        pos_stream_task,
        "pos_stream",
        4096,
        nullptr,
        4,
        nullptr);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "failed to create POS stream task");
        return ESP_ERR_NO_MEM;
    }

    g_started = true;
    ESP_LOGD(TAG, "serial RX and POS streaming tasks started");
    return ESP_OK;
}

esp_err_t send_boot_message()
{
    TxLock lock;
    const int written = std::printf("OK_INIT_ROBOT\n");
    std::fflush(stdout);

    return written > 0 ? ESP_OK : ESP_FAIL;
}

} // namespace serial_protocol
} // namespace robo_6dof
