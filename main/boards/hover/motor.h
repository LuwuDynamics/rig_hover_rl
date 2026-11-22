#ifndef __MOTOR_H
#define __MOTOR_H

/* Hover 组态：云台 EM3（ID=3）+ 轮毂 KP4012。协议见 em3.h / kp4012.h */

#include <stdint.h>
#include <stddef.h>
#include <driver/uart.h>

void SendMotorCommand(uint8_t *pData, uint16_t size);
void WriteByte_P_V(uint8_t ID, short pos, short vel);
void EnableServo(uint8_t ID, uint8_t mode);
void ReadMotorState(uint8_t ID);
void ReadServoVoltage(uint8_t ID);
void ReadWheelState(uint8_t readID);
void sendWheelTor(short tor1, short tor2);
void motor_rx();
// Consistent snapshot of the wheel odometry (wheel1_x / wheel2_x, forward positive, 1024 counts per turn) and of the time
// (esp_timer, us) each wheel's state frame was last received (0 = never). Used by the RL controller to differentiate positions.
void GetWheelSnapshot(float* x1, float* x2, int64_t* us1, int64_t* us2);

inline constexpr int64_t kBatteryVoltagePollIntervalUs = 1000000;

struct BatteryVoltageSample {
    float volts;
    int64_t received_us;

    int LevelPercent() const;
};

// The pack voltage is reported by the head servo in 0.1 V steps.
BatteryVoltageSample GetBatteryVoltageSample();

#endif
