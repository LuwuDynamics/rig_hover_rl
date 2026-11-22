#include "motor.h"
#include "robot.h"
#include "em3.h"
#include "kp4012.h"
#include "scs_bus.h"
#include <algorithm>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

static portMUX_TYPE s_battery_mux = portMUX_INITIALIZER_UNLOCKED;
static BatteryVoltageSample s_battery_voltage{};

BatteryVoltageSample GetBatteryVoltageSample() {
    portENTER_CRITICAL(&s_battery_mux);
    const auto sample = s_battery_voltage;
    portEXIT_CRITICAL(&s_battery_mux);
    return sample;
}

int BatteryVoltageSample::LevelPercent() const {
    constexpr float kEmptyVolts = 6.6f;
    constexpr float kFullVolts = 8.4f;
    if (volts < 0.1f) {
        return -1;
    }
    const float level = (volts - kEmptyVolts) / (kFullVolts - kEmptyVolts);
    return static_cast<int>(std::clamp(level, 0.0f, 1.0f) * 100.0f);
}

// wheelN_x is written by motor_rx_task and read by robot_task: keep (counts, time of the update) consistent
static portMUX_TYPE s_wheel_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_wheel1_us = 0;
static int64_t s_wheel2_us = 0;

void GetWheelSnapshot(float* x1, float* x2, int64_t* us1, int64_t* us2) {
    portENTER_CRITICAL(&s_wheel_mux);
    *x1 = wheel1_x;
    *x2 = wheel2_x;
    *us1 = s_wheel1_us;
    *us2 = s_wheel2_us;
    portEXIT_CRITICAL(&s_wheel_mux);
}

static float K_V = 60.0f * 3.1413f / 1024.0f;

static void OnBusFrame(uint8_t id, const uint8_t *frame, uint8_t length_field) {
    switch (id) {
        case 1:
        case 11: {
            int16_t pos = 0, vel = 0;
            if (!kp4012_decode_state(frame, length_field, &pos, &vel)) {
                return;
            }
            float temp_spd = 4.5f * (float)vel * K_V;
            wheel1_vel = lp_vel * wheel1_vel + (1 - lp_vel) * temp_spd;
            int temp_pos = pos;
            const int64_t now_us = esp_timer_get_time();
            portENTER_CRITICAL(&s_wheel_mux);
            if (temp_pos - last11Pos < -800) {
                wheel1_x += 1023 + temp_pos - last11Pos;
            } else if (temp_pos - last11Pos > 800) {
                wheel1_x += -1023 + temp_pos - last11Pos;
            } else {
                wheel1_x += temp_pos - last11Pos;
            }
            s_wheel1_us = now_us;
            portEXIT_CRITICAL(&s_wheel_mux);
            last11Pos = temp_pos;
            break;
        }
        case 2:
        case 21: {
            int16_t pos = 0, vel = 0;
            if (!kp4012_decode_state(frame, length_field, &pos, &vel)) {
                return;
            }
            float temp_spd = 4.5f * (float)(-vel) * K_V;
            wheel2_vel = lp_vel * wheel2_vel + (1 - lp_vel) * temp_spd;
            int temp_pos = pos;
            const int64_t now_us = esp_timer_get_time();
            portENTER_CRITICAL(&s_wheel_mux);
            if (temp_pos - last21Pos < -800) {
                wheel2_x -= 1023 + temp_pos - last21Pos;
            } else if (temp_pos - last21Pos > 800) {
                wheel2_x -= -1023 + temp_pos - last21Pos;
            } else {
                wheel2_x -= temp_pos - last21Pos;
            }
            s_wheel2_us = now_us;
            portEXIT_CRITICAL(&s_wheel_mux);
            last21Pos = temp_pos;
            break;
        }
        case 3: {
            float volts = 0.0f;
            if (scs_decode_voltage(frame, length_field, &volts)) {
                const int64_t now_us = esp_timer_get_time();
                portENTER_CRITICAL(&s_battery_mux);
                s_battery_voltage = {volts, now_us};
                portEXIT_CRITICAL(&s_battery_mux);
                break;
            }
            int16_t pos = 0;
            if (em3_decode_state(frame, length_field, &pos, nullptr, nullptr)) {
                head_fb_pos = pos;
                q_head = (head_zero_pos - pos) / 10.0f;
            }
            break;
        }
        default:
            break;
    }
}

void SendMotorCommand(uint8_t *pData, uint16_t size) {
    uart_write_bytes(UART_NUM_2, pData, size);
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(50));
}

void ReadMotorState(uint8_t ID) {
    em3_read_state(ID);
}

void motor_rx() {
    static bool registered = false;
    if (!registered) {
        scs_bus_on_frame(OnBusFrame);
        registered = true;
    }
    scs_bus_poll(0);
}

void WriteByte_P_V(uint8_t ID, short pos, short vel) {
    em3_write_pos_vel(ID, pos, vel);
}

void EnableServo(uint8_t ID, uint8_t mode) {
    em3_enable(ID, mode);
}

void ReadServoVoltage(uint8_t ID) {
    em3_read_voltage(ID);
}

void ReadWheelState(uint8_t readID) {
    kp4012_read_state(readID);
}

void sendWheelTor(short tor1, short tor2) {
    const uint8_t ids[6] = {55, 1, 2, 11, 21, 66};
    const int16_t pos[6] = {0, 0, 0, 0, 0, 0};

    int16_t tor11 = tor1;
    int16_t tor21 = tor2;
    const int64_t now_us = esp_timer_get_time();
    if (motor11_test_until_us > now_us) {
        tor11 = motor11_test_torque;
    }
    if (motor21_test_until_us > now_us) {
        tor21 = motor21_test_torque;
    }

    const int16_t torque[6] = {0, tor1, tor2, tor11, tor21, 0};
    kp4012_sync_write_pos_torque(6, ids, pos, torque);
}
