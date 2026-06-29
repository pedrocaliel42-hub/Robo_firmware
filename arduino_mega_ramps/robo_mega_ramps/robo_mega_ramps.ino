#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t kAxisCount = 5;
constexpr uint8_t kLineBufferLength = 128;
constexpr uint32_t kBaudRate = 115200;
constexpr uint16_t kStepPulseHighUs = 5;
// Perfil de velocidade dos movimentos coordenados:
// - parte de kStepIntervalStartUs (mais lento);
// - acelera durante kAccelerationSteps;
// - permanece em kStepIntervalCruiseUs;
// - desacelera de forma simetrica antes do destino.
//
// Intervalo menor = velocidade maior. Calibre o cruzeiro gradualmente para
// evitar perda de passos. O pulso HIGH e o tempo do proprio loop se somam
// ao intervalo configurado.
constexpr uint16_t kStepIntervalStartUs = 800;
constexpr uint16_t kStepIntervalCruiseUs = 120;
constexpr unsigned long kAccelerationSteps = 1200UL;
static_assert(
    kStepIntervalStartUs >= kStepIntervalCruiseUs,
    "O intervalo inicial deve ser maior ou igual ao intervalo de cruzeiro");
static_assert(kAccelerationSteps > 0UL, "A rampa deve possuir ao menos um passo");
constexpr uint8_t kGripperServoPin = 11;  // RAMPS 1.4 servo header (AUX-3, D11)

// Ajuste aqui as reducoes mecanicas dos eixos no RAMPS.
#ifndef ROBO_Q1_GEAR_RATIO
#define ROBO_Q1_GEAR_RATIO 38.4F
#endif
#ifndef ROBO_Q2_GEAR_RATIO
#define ROBO_Q2_GEAR_RATIO 38.4F
#endif
#ifndef ROBO_Q3_GEAR_RATIO
#define ROBO_Q3_GEAR_RATIO 38.4F
#endif
#ifndef ROBO_Q4_GEAR_RATIO
#define ROBO_Q4_GEAR_RATIO 38.4F
#endif
#ifndef ROBO_Q5_GEAR_RATIO
#define ROBO_Q5_GEAR_RATIO 38.4F
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
    {"q3/Z", 46, 48, 62, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q3_GEAR_RATIO, false},
    {"q4/E0", 26, 28, 24, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q4_GEAR_RATIO, false},
    {"q5/E1", 36, 34, 30, -180.0F, 180.0F, 0.0F, 200, 16, ROBO_Q5_GEAR_RATIO, false},
};

float g_current_deg[kAxisCount] = {};
float g_prepared_deg[kAxisCount] = {};
uint32_t g_prepared_sequence = 0;
bool g_has_prepared_move = false;
bool g_estop = false;
bool g_stop_requested = false;
bool g_referenced = false;
bool g_q2_locked = true;

char g_line[kLineBufferLength] = {};
uint8_t g_line_length = 0;

Servo g_gripper_servo;

void enable_drivers(bool enabled)
{
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        digitalWrite(kAxes[axis].enable_pin, enabled ? LOW : HIGH);
    }
}

void enable_q2_q3(bool enabled)
{
    digitalWrite(kAxes[1].enable_pin, enabled ? LOW : HIGH);
    digitalWrite(kAxes[2].enable_pin, enabled ? LOW : HIGH);
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

float steps_to_degrees(uint8_t axis, long steps)
{
    return static_cast<float>(steps) / steps_per_degree(kAxes[axis]);
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

uint16_t step_interval_us(unsigned long step_index, unsigned long total_steps)
{
    if (total_steps <= 1UL) {
        return kStepIntervalStartUs;
    }

    // A distancia ate a extremidade mais proxima gera perfis simetricos de
    // aceleracao e desaceleracao. Em movimentos curtos, as duas rampas se
    // encontram antes do cruzeiro, formando naturalmente um perfil triangular.
    const unsigned long steps_from_start = step_index;
    const unsigned long steps_to_end = total_steps - 1UL - step_index;
    unsigned long ramp_progress =
        steps_from_start < steps_to_end ? steps_from_start : steps_to_end;
    if (ramp_progress > kAccelerationSteps) {
        ramp_progress = kAccelerationSteps;
    }

    const uint32_t interval_range =
        static_cast<uint32_t>(kStepIntervalStartUs - kStepIntervalCruiseUs);
    const uint32_t interval_reduction =
        (interval_range * ramp_progress) / kAccelerationSteps;

    return static_cast<uint16_t>(
        static_cast<uint32_t>(kStepIntervalStartUs) - interval_reduction);
}

void send_line(const __FlashStringHelper* text)
{
    Serial1.println(text);
    Serial.println(text);
}

void send_error(const __FlashStringHelper* error)
{
    Serial1.println(error);
    Serial.println(error);
}

void send_sequence_response(const char* prefix, uint32_t sequence)
{
    Serial1.print(prefix);
    Serial1.print(',');
    Serial1.println(sequence);
    Serial.print(prefix);
    Serial.print(',');
    Serial.println(sequence);
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
                g_referenced = false;
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
        enable_q2_q3(false);
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

        delayMicroseconds(step_interval_us(step_index, max_steps));
    }

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        g_current_deg[axis] = g_prepared_deg[axis];
    }

    enable_q2_q3(false);
    return true;
}

void handle_mprep(char* tokens[], uint8_t token_count)
{
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }
    if (!g_referenced) {
        send_error(F("MERR_UNREFERENCED"));
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
    if (g_q2_locked && fabs(parsed_targets[1] - g_current_deg[1]) > 0.001F) {
        send_error(F("MERR_Q2_LOCKED"));
        return;
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

void handle_mgrp(char* tokens[], uint8_t token_count)
{
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }

    if (token_count != 2) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    float parsed = 0.0F;
    if (!parse_float_token(tokens[1], &parsed)) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    const int percent = constrain(static_cast<int>(parsed), 0, 100);
    // Mapeia 0-100% para 2000-1000 µs (invertido: 0%=aberta, 100%=fechada)
    const int us = map(percent, 0, 100, 2000, 1000);
    g_gripper_servo.writeMicroseconds(us);
    send_line(F("MOK_GRIP"));
}

void handle_mref(uint8_t token_count)
{
    if (token_count != 1) {
        send_error(F("MERR_FORMAT"));
        return;
    }
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        g_current_deg[axis] = kAxes[axis].home_deg;
        g_prepared_deg[axis] = kAxes[axis].home_deg;
    }
    g_has_prepared_move = false;
    g_stop_requested = false;
    g_referenced = true;
    send_line(F("MOK_REF_HOME"));
}

void handle_mpower23(char* tokens[], uint8_t token_count)
{
    if (token_count != 2) {
        send_error(F("MERR_FORMAT"));
        return;
    }
    if (strcmp(tokens[1], "ON") == 0) {
        enable_q2_q3(true);
        send_line(F("MOK_Q23_ON"));
    } else if (strcmp(tokens[1], "OFF") == 0) {
        enable_q2_q3(false);
        send_line(F("MOK_Q23_OFF"));
    } else {
        send_error(F("MERR_FORMAT"));
    }
}

void handle_mq2lock(char* tokens[], uint8_t token_count)
{
    if (token_count != 2) {
        send_error(F("MERR_FORMAT"));
        return;
    }
    if (strcmp(tokens[1], "ON") == 0) {
        g_q2_locked = true;
        send_line(F("MOK_Q2_LOCKED"));
    } else if (strcmp(tokens[1], "OFF") == 0) {
        g_q2_locked = false;
        send_line(F("MOK_Q2_UNLOCKED"));
    } else {
        send_error(F("MERR_FORMAT"));
    }
}

void handle_mjog(char* tokens[], uint8_t token_count)
{
    if (g_estop) {
        send_error(F("MERR_ESTOP"));
        return;
    }

    // MJOG + 5 deltas (graus, relativo)
    if (token_count != 6) {
        send_error(F("MERR_FORMAT"));
        return;
    }

    long delta_steps[kAxisCount] = {};
    unsigned long abs_steps[kAxisCount] = {};
    unsigned long accumulators[kAxisCount] = {};
    unsigned long max_steps = 0;
    float reached_deg[kAxisCount] = {};

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        float delta_deg = 0.0F;
        if (!parse_float_token(tokens[axis + 1], &delta_deg)) {
            send_error(F("MERR_FORMAT"));
            return;
        }
        delta_steps[axis] = degrees_to_steps(axis, delta_deg);
        if (axis == 1 && g_q2_locked && delta_steps[axis] != 0) {
            send_error(F("MERR_Q2_LOCKED"));
            return;
        }
        reached_deg[axis] = g_current_deg[axis] + steps_to_degrees(axis, delta_steps[axis]);
        if (g_referenced && !validate_target(axis, reached_deg[axis])) {
            send_error(F("MERR_LIMIT"));
            return;
        }
        abs_steps[axis] = labs(delta_steps[axis]);
        if (abs_steps[axis] > max_steps) {
            max_steps = abs_steps[axis];
        }
        set_direction(axis, delta_steps[axis] >= 0);
    }

    g_stop_requested = false;

    if (max_steps == 0) {
        enable_q2_q3(false);
        send_line(F("MJOGDONE"));
        return;
    }

    enable_drivers(true);

    for (unsigned long step_index = 0; step_index < max_steps; ++step_index) {
        poll_runtime_commands();
        if (g_estop || g_stop_requested) {
            enable_drivers(false);
            send_error(g_estop ? F("MERR_ESTOP") : F("MERR_FAULT"));
            return;
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

        delayMicroseconds(step_interval_us(step_index, max_steps));
    }

    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        g_current_deg[axis] = reached_deg[axis];
    }

    enable_q2_q3(false);
    send_line(F("MJOGDONE"));
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
        g_referenced = false;
        g_has_prepared_move = false;
        enable_drivers(false);
        send_line(F("MOK_ESTOP"));
    } else if (strcmp(tokens[0], "MGRP") == 0) {
        handle_mgrp(tokens, token_count);
    } else if (strcmp(tokens[0], "MREF") == 0) {
        handle_mref(token_count);
    } else if (strcmp(tokens[0], "MPOWER23") == 0) {
        handle_mpower23(tokens, token_count);
    } else if (strcmp(tokens[0], "MQ2LOCK") == 0) {
        handle_mq2lock(tokens, token_count);
    } else if (strcmp(tokens[0], "MJOG") == 0) {
        handle_mjog(tokens, token_count);
    } else {
        send_error(F("MERR_FORMAT"));
    }
}

char g_usb_line[kLineBufferLength] = {};
uint8_t g_usb_line_length = 0;

void setup()
{
    Serial.begin(kBaudRate);
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

    // Servo da garra no header RAMPS 1.4 (D11)
    g_gripper_servo.attach(kGripperServoPin, 1000, 2000);
    g_gripper_servo.writeMicroseconds(2000);  // posição inicial: aberta (0%)
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

    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());

        if (ch == '\r' || ch == '\n') {
            if (g_usb_line_length == 0) {
                continue;
            }

            g_usb_line[g_usb_line_length] = '\0';
            handle_line(g_usb_line);
            g_usb_line_length = 0;
            continue;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (g_usb_line_length > 0) {
                --g_usb_line_length;
            }
            continue;
        }

        if (!isPrintable(ch)) {
            continue;
        }

        if (g_usb_line_length >= kLineBufferLength - 1) {
            g_usb_line_length = 0;
            send_error(F("MERR_FORMAT"));
            continue;
        }

        g_usb_line[g_usb_line_length++] = ch;
    }
}
