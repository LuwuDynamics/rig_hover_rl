#ifndef _IMU_H_
#define _IMU_H_
#include "sdkconfig.h"
#include "robot.h"
// IMU (QMI8658C) 接口
void imu_init();
void imu_read_once();
void imu_deinit();
bool imu_is_initialized();

// 姿态数据
extern float roll;
extern float pitch;
extern float yaw;
extern float accel_x;
extern float accel_y;
extern float accel_z;
extern float pit_gyro;
extern float dq;

#if defined(CONFIG_BOARD_TYPE_HOVER)
// Samples normally arrive every few milliseconds. The RL control loop also treats
// a gap longer than 40 ms as a discontinuity.
inline constexpr int64_t kImuMaxGapUs = 40000;
struct ImuAttitude {
    int64_t sample_us;
    int64_t continuous_since_us;
    float pitch, roll, yaw;
    float pitch_rate_dps, yaw_rate_dps;
    float raw_acc_x, raw_acc_y, raw_acc_z;
    float raw_gyro_x, raw_gyro_y, raw_gyro_z;
};
ImuAttitude imu_get_attitude();

// Update the head-frame IMU mounting tilt compensation (degrees, clamped internally).
void imu_set_head_mount_deg(float pitch_deg, float roll_deg);

// Update online roll/pitch gyro zero biases [deg/s] (gyro_x, gyro_z); yaw bias stays from boot calibration.
void imu_set_gyro_rp_bias(float bx, float bz);
void imu_get_gyro_bias(float bias_dps[3]);

// Hover only (RL controller): gyro yaw rate in deg/s, zero-bias compensated, positive in the direction in which `yaw` increases.
float imu_yaw_rate_dps();
#endif
#endif
