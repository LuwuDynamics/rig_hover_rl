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
// Hover only (RL controller): gyro yaw rate in deg/s, zero-bias compensated, positive in the direction in which `yaw` increases.
float imu_yaw_rate_dps();
#endif
#endif
