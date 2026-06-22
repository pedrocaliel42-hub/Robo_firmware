#include <Arduino.h>

constexpr uint8_t kAxisCount = 5;
constexpr uint8_t kLineBufferLength = 128;
constexpr uint32_t kBaudRate = 115200;
constexpr uint16_t kStepPulseHighUs = 5;
constexpr uint16_t kStepIntervalUs = 2000;

// Ajuste aqui as reducoes mecanicas dos eixos no RAMPS.
// Exemplo: motor com reducao 10:1 usa 10.0F.
#ifndef ROBO_Q1_GEAR_RATIO
#define ROBO_Q1_GEAR_RATIO 1.0F
#endif
#ifndef ROBO_Q2_GEAR_RATIO
#define ROBO_Q2_GEAR_RATIO 1.0F
#endif
#ifndef ROBO_Q3_GEAR_RATIO
#define ROBO_Q3_GEAR_RATIO 1.0F
#endif
#ifndef ROBO_Q4_GEAR_RATIO
#define ROBO_Q4_GEAR_RATIO 1.0F
#endif
#ifndef ROBO_Q5_GEAR_RATIO
#define ROBO_Q5_GEAR_RATIO 1.0F
#endif

struct AxisConfig {
    const char* name;
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    float min_deg;
    float max_deg;
    float home_deg;
    int steps_per_rev;
    int microstep;
    float gear_ratio;
    bool invert_dir;
};

constexpr AxisConfig kAxes[kAxisCount] = {
    {"q1/X", 54, 55, 38, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q1_GEAR_RATIO, false},
    {"q2/Y", 60, 61, 56, -90.0F, 90.0F, 0.0F, 200, 16, ROBO_Q2_GEAR_RATIO, false},
    {"q3/Z", 46, 48, 62, -180.0F, 180.0F, 90.0F, 200, 16, ROBO_Q3_GEAR_RATIO, false},
    {"q4/E0", 26, 28, 24, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q4_GEAR_RATIO, false},
    {"q5/E1", 36, 34, 30, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q5_GEAR_RATIO, false},
};

float g_current_deg[kAxisCount] = {};
float g_prepared_deg[kAxisCount] = {};
uint32_t g_prepared_sequence = 0;
bool g_has_prepared_move = false;
bool g_estop = false;
bool g_stop_requested = false;

char g_line[kLineBufferLength] = {};
uint8_t g_line_length = 0;

void enable_drivers(bool enabled)
{
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        digitalWrite(kAxes[axis].enable_pin, enabled ? LOW : HIGH);
    }
}

float steps_per_degree(const AxisConfig& axis)
{
    return (static_cast<float>(axis.steps_per_rev) *
            static_cast<float>(axis.microstep) *
            axis.gear_ratio) /
           360.0F;
}

long degrees_to_steps(uint8_t axis, float degrees)
{
    return lround(degrees * steps_per_degree(kAxes[axis]));
}

bool validate_target(uint8_t axis, float target_deg)
{
    if (!isfinite(target_deg)) {
        return false;
    }
    return target_deg >= kAxes[axis].min_deg && target_deg <= kAxes[axis].max_deg;
}

void set_direction(uint8_t axis, bool positive_direction)
{
    const bool level = kAxes[axis].invert_dir ? !positive_direction : positive_direction;
    digitalWrite(kAxes[axis].dir_pin, level ? HIGH : LOW);
}

void pulse_axis(uint8_t axis)
{
    digitalWrite(kAxes[axis].step_pin, HIGH);
    delayMicroseconds(kStepPulseHighUs);
    digitalWrite(kAxes[axis].step_pin, LOW);
}

void send_line(const __FlashStringHelper* text)
{
    Serial1.println(text);
}

void send_error(const __FlashStringHelper* error)
{
    Serial1.println(error);
}

void send_sequence_response(const char* prefix, uint32_t sequence)
{
    Serial1.print(prefix);
    Serial1.print(',');
    Serial1.println(sequence);
}

char* trim_token(char* token)
{
    while (*token != '\0' && isspace(*token)) {
        ++token;
    }

    char* end = token + strlen(token);
    while (end > token && isspace(*(end - 1))) {
        --end;
        *end = '\0';
    }

    return token;
}

uint8_t split_csv(char* line, char* tokens[], uint8_t max_tokens)
{
    uint8_t count = 0;
    char* cursor = line;

    while (true) {
        if (count >= max_tokens) {
            return max_tokens + 1;
        }

        tokens[count++] = trim_token(cursor);
        char* comma = strchr(cursor, ',');
        if (comma == nullptr) {
            break;
        }

        *comma = '\0';
        tokens[count - 1] = trim_token(cursor);
        cursor = comma + 1;
    }

    return count;
}

bool parse_float_token(const char* token, float* value)
{
    if (token == nullptr || token[0] == '\0' || value == nullptr) {
        return false;
    }

    char* end = nullptr;
    const float parsed = strtod(token, &end);
    if (end == token || end == nullptr || *end != '\0' || !isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

bool parse_sequence(const char* token, uint32_t* sequence)
{
    if (token == nullptr || token[0] == '\0' || sequence == nullptr) {
        return false;
    }

    char* end = nullptr;
    const unsigned long parsed = strtoul(token, &end, 10);
    if (end == token || end == nullptr || *end != '\0') {
        return false;
    }

    *sequence = static_cast<uint32_t>(parsed);
    return true;
}

void poll_runtime_commands()
{
    while (Serial1.available() > 0) {
        const char ch = static_cast<char>(Serial1.read());
        if (ch == '\n' || ch == '\r') {
            g_line[g_line_length] = '\0';
            if (strcmp(g_line, "MESTOP") == 0) {
                g_estop = true;
                g_has_prepared_move = false;
                enable_drivers(false);
                send_line(F("MOK_ESTOP"));
            } else if (strncmp(g_line, "MSTOP", 5) == 0) {
                g_stop_requested = true;
                g_has_prepared_move = false;
                enable_drivers(false);
                send_sequence_response("MSTOPPED", g_prepared_sequence);
            }
            g_line_length = 0;
            continue;
        }

        if (isPrintable(ch) && g_line_length < kLineBufferLength - 1) {
            g_line[g_line_length++] = ch;
        }
    }
}

bool execute_prepared_move()
{
    long delta_steps[kAxisCount] = {};
    unsigned long abs_steps[kAxisCount] = {};
    unsigned long accumulators[kAxisCount] = {};
    unsigned long max_steps = 0;

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        const long current_steps = degrees_to_steps(axis, g_current_deg[axis]);
        const long target_steps = degrees_to_steps(axis, g_prepared_deg[axis]);
        delta_steps[axis] = target_steps - current_steps;
        abs_steps[axis] = labs(delta_steps[axis]);
        if (abs_steps[axis] > max_steps) {
            max_steps = abs_steps[axis];
        }
        set_direction(axis, delta_steps[axis] >= 0);
    }

    if (max_steps == 0) {
        return true;
    }

    enable_drivers(true);

    for (unsigned long step_index = 0; step_index < max_steps; ++step_index) {
        poll_runtime_commands();
        if (g_estop || g_stop_requested) {
            enable_drivers(false);
            return false;
        }

        for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
            if (abs_steps[axis] == 0) {
                continue;
            }

            accumulators[axis] += abs_steps[axis];
            if (accumulators[axis] >= max_steps) {
                accumulators[axis] -= max_steps;
                pulse_axis(axis);
            }
        }

        delayMicroseconds(kStepIntervalUs);
    }

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        g_current_deg[axis] = g_prepared_deg[axis];
    }

    return true;
}

void handle_mprep(char* tokens[], uint8_t token_count)
{
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }

    if (token_count != 7) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    uint32_t sequence = 0;
    if (!parse_sequence(tokens[1], &sequence)) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    float parsed_targets[kAxisCount] = {};
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        if (!parse_float_token(tokens[axis + 2], &parsed_targets[axis])) {
            send_error(F("MERR_FORMAT"));
            return;
        }
        if (!validate_target(axis, parsed_targets[axis])) {
            send_error(F("MERR_LIMIT"));
            return;
        }
    }

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        g_prepared_deg[axis] = parsed_targets[axis];
    }
    g_prepared_sequence = sequence;
    g_has_prepared_move = true;
    g_stop_requested = false;

    send_sequence_response("MREADY", sequence);
}

void handle_mgo(char* tokens[], uint8_t token_count)
{
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }

    if (token_count != 2) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    uint32_t sequence = 0;
    if (!parse_sequence(tokens[1], &sequence)) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    if (!g_has_prepared_move || sequence != g_prepared_sequence) {
        send_error(F("MERR_BUSY"));
        return;
    }

    const bool ok = execute_prepared_move();
    g_has_prepared_move = false;

    if (ok) {
        send_sequence_response("MDONE", sequence);
    } else if (g_estop) {
        send_error(F("MERR_ESTOP"));
    } else if (g_stop_requested) {
        g_stop_requested = false;
    } else {
        send_error(F("MERR_FAULT"));
    }
}

void handle_mstop(char* tokens[], uint8_t token_count)
{
    uint32_t sequence = g_prepared_sequence;
    if (token_count == 2) {
        parse_sequence(tokens[1], &sequence);
    }

    g_has_prepared_move = false;
    g_stop_requested = true;
    enable_drivers(false);
    send_sequence_response("MSTOPPED", sequence);
}

void handle_line(char* line)
{
    char* tokens[8] = {};
    const uint8_t token_count = split_csv(line, tokens, 8);
    if (token_count == 0 || token_count > 8 || tokens[0][0] == '\0') {
        send_error(F("MERR_FORMAT"));
        return;
    }

    if (strcmp(tokens[0], "MPING") == 0) {
        send_line(F("MOK_PONG"));
    } else if (strcmp(tokens[0], "MPREP") == 0) {
        handle_mprep(tokens, token_count);
    } else if (strcmp(tokens[0], "MGO") == 0) {
        handle_mgo(tokens, token_count);
    } else if (strcmp(tokens[0], "MSTOP") == 0) {
        handle_mstop(tokens, token_count);
    } else if (strcmp(tokens[0], "MESTOP") == 0) {
        g_estop = true;
        g_has_prepared_move = false;
        enable_drivers(false);
        send_line(F("MOK_ESTOP"));
    } else {
        send_error(F("MERR_FORMAT"));
    }
}

void setup()
{
    Serial1.begin(kBaudRate);

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        pinMode(kAxes[axis].step_pin, OUTPUT);
        pinMode(kAxes[axis].dir_pin, OUTPUT);
        pinMode(kAxes[axis].enable_pin, OUTPUT);
        digitalWrite(kAxes[axis].step_pin, LOW);
        digitalWrite(kAxes[axis].dir_pin, LOW);
        g_current_deg[axis] = kAxes[axis].home_deg;
        g_prepared_deg[axis] = kAxes[axis].home_deg;
    }

    enable_drivers(false);
}

void loop()
{
    while (Serial1.available() > 0) {
        const char ch = static_cast<char>(Serial1.read());

        if (ch == '\r' || ch == '\n') {
            if (g_line_length == 0) {
                continue;
            }

            g_line[g_line_length] = '\0';
            handle_line(g_line);
            g_line_length = 0;
            continue;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (g_line_length > 0) {
                --g_line_length;
            }
            continue;
        }

        if (!isPrintable(ch)) {
            continue;
        }

        if (g_line_length >= kLineBufferLength - 1) {
            g_line_length = 0;
            send_error(F("MERR_FORMAT"));
            continue;
        }

        g_line[g_line_length++] = ch;
    }
}
