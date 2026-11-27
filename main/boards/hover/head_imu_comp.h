// head_imu_comp.h -- 3D head-frame IMU mounting compensation and online mount tilt estimator for RIG-Hover.
//
// Header-only, no ESP-IDF dependencies (<math.h>/<stdint.h> only), so the exact same implementation is compiled into
// the firmware (imu.cc / robot.cc) and verified in host unit tests and closed-loop MuJoCo simulations.
//
// Why this exists:
//   The QMI8658C IMU is mounted inside the rotating head. Any fixed mounting tilt of the PCB/IMU relative to the
//   vertical head-servo axis (especially roll tilt) rotates with q_head when projected into the body frame:
//     pit_acc(q) ~ pitch_body + p_mount * cos(q) - r_mount * sin(q)
//     pit_gyro(q) also picks up d/dt(p_mount * cos(q) - r_mount * sin(q)) while the head turns.
//   Subtracting only imu_zero * cos(q) leaves the -r_mount * sin(q) error and the gyro projection uncompensated,
//   causing the balance controller to drive the wheels forward/backward whenever the head turns.
//
// How it works:
//   1. Rotate both the head-frame accelerometer and bias-compensated gyroscope by R_hi(p_mount, r_mount) =
//      Ry(p_mount) * Rx(-r_mount) BEFORE rotating by q_head into the body frame.
//   2. Project the leveled head-frame signals into the body frame with an exact SO(2) rotation by q_head.
//   3. Add p_mount back to pit_acc AFTER the q_head projection so pitch at q_head == 0 stays anchored identically
//      to the raw sensor reading (zero feedback loop while p_mount adapts, full compatibility with imu_zero = 4 deg).
//   4. While the robot is quietly balancing on the floor with |q_head| < 15 deg, slowly track (p_mount, r_mount)
//      directly from the raw head-frame gravity vector.
#pragma once
#include <math.h>
#include <stdint.h>

namespace hover_rl {

inline constexpr float kDefaultHeadMountPitchDeg = 4.0f;
inline constexpr float kDefaultHeadMountRollDeg = 0.0f;
inline constexpr float kHeadMountLimitDeg = 12.0f;

struct HeadImuProjection {
    float pitch_acc_deg = 0.0f;
    float roll_acc_deg = 0.0f;
    float pitch_gyro_dps = 0.0f;
};

inline float ClampMountDeg(float deg) {
    if (deg < -kHeadMountLimitDeg) return -kHeadMountLimitDeg;
    if (deg > kHeadMountLimitDeg) return kHeadMountLimitDeg;
    return deg;
}

// Compute the exact (pitch_deg, roll_deg) head-frame mounting tilt from a quiet raw accelerometer sample
// in imu.cc sign conventions (ax ~ +9.8 up, ay ~ +g*sin(p), az ~ +g*sin(r)*cos(p)).
inline bool HeadMountFromRawAccel(float ax, float ay, float az, float* pitch_deg, float* roll_deg) {
    if (!__builtin_isfinite(ax) || !__builtin_isfinite(ay) || !__builtin_isfinite(az)) {
        return false;
    }
    if (ax <= 7.5f) {
        return false;
    }
    const float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (a_norm < 8.5f || a_norm > 11.2f) {
        return false;
    }
    constexpr float kRadToDeg = 57.2957795f;
    const float r = atan2f(az, ax) * kRadToDeg;
    const float p = atan2f(ay, sqrtf(ax * ax + az * az)) * kRadToDeg;
    if (!__builtin_isfinite(p) || !__builtin_isfinite(r)) {
        return false;
    }
    *pitch_deg = ClampMountDeg(p);
    *roll_deg = ClampMountDeg(r);
    return true;
}

// Undo 3D head-frame IMU mounting tilt on both accelerometer and bias-compensated gyroscope,
// then project into the body frame via an exact SO(2) rotation by q_head_deg.
//
// imu.cc head-frame conventions:
//   ax = -accx_scaled (+head_up), ay = -accy_scaled (-head_fwd), az = +accz_scaled (-head_left)
//   gx = gyro_x - gyro_bias[0] (+roll_rate about head_fwd)
//   gy = gyro_y - gyro_bias[1] (-yaw_rate about head_up)
//   gz = gyro_z - gyro_bias[2] (-pitch_rate about head_left)
inline HeadImuProjection ProjectHeadImu(float ax, float ay, float az,
                                        float gx, float gy, float gz,
                                        float q_head_deg,
                                        float mount_pitch_deg,
                                        float mount_roll_deg) {
    constexpr float kDegToRad = 0.0174532925f;
    constexpr float kRadToDeg = 57.2957795f;

    const float pm_deg = ClampMountDeg(mount_pitch_deg);
    const float rm_deg = ClampMountDeg(mount_roll_deg);
    const float pm = pm_deg * kDegToRad;
    const float rm = rm_deg * kDegToRad;
    const float cp = cosf(pm), sp = sinf(pm);
    const float cr = cosf(rm), sr = sinf(rm);

    // Right-handed IMU vector v_imu = (vx, vy, vz) in (fwd, left, up):
    //   R_hi = Ry(+pm) * Rx(-rm):
    //     x1 = vx
    //     y1 =  cr * vy + sr * vz
    //     z1 = -sr * vy + cr * vz
    //     xh =  cp * x1 + sp * z1
    //     yh =  y1
    //     zh = -sp * x1 + cp * z1
    auto level = [cp, sp, cr, sr](float vx, float vy, float vz, float& xh, float& yh, float& zh) {
        const float y1 = cr * vy + sr * vz;
        const float z1 = -sr * vy + cr * vz;
        xh = cp * vx + sp * z1;
        yh = y1;
        zh = -sp * vx + cp * z1;
    };

    float fx_h, fy_h, fz_h;
    level(-ay, -az, ax, fx_h, fy_h, fz_h);
    const float ax_l = fz_h;
    const float ay_l = -fx_h;
    const float az_l = -fy_h;

    float wx_h, wy_h, wz_h;
    level(gx, -gz, -gy, wx_h, wy_h, wz_h);
    const float gx_l = wx_h;
    const float gz_l = -wy_h;

    const float qr = q_head_deg * kDegToRad;
    const float cq = cosf(qr);
    const float sq = sinf(qr);

    HeadImuProjection out;
    out.pitch_gyro_dps = sq * gx_l - cq * gz_l;
    out.pitch_acc_deg = atan2f(cq * ay_l - sq * az_l, ax_l) * kRadToDeg + pm_deg;
    out.roll_acc_deg = atan2f(cq * az_l + sq * ay_l, ax_l) * kRadToDeg;
    return out;
}

// Online head-frame mounting tilt tracker.
// Only adapts after the robot has stood quietly in balance with the head near center (|q_head| < 15 deg).
// Preserves the learned mounting tilt across pickups / restarts; resetQuiet() only clears the streak timer.
class HeadMountEstimator {
public:
    static constexpr float kMinQuietS = 0.4f;
    static constexpr float kFastPhaseS = 2.0f;
    static constexpr float kFastTauS = 0.6f;
    static constexpr float kSlowTauS = 2.5f;

    bool update(float dt_s, bool quiet, float raw_ax, float raw_ay, float raw_az) {
        if (!quiet || dt_s <= 0.0f || dt_s > 0.1f) {
            quiet_s_ = 0.0f;
            return false;
        }
        quiet_s_ += dt_s;
        if (quiet_s_ < kMinQuietS) {
            return false;
        }
        float p_meas = 0.0f, r_meas = 0.0f;
        if (!HeadMountFromRawAccel(raw_ax, raw_ay, raw_az, &p_meas, &r_meas)) {
            return false;
        }
        learned_s_ += dt_s;
        const float tau = learned_s_ < kFastPhaseS ? kFastTauS : kSlowTauS;
        float alpha = dt_s / tau;
        if (alpha > 0.1f) alpha = 0.1f;
        pitch_deg_ = ClampMountDeg(pitch_deg_ + alpha * (p_meas - pitch_deg_));
        roll_deg_ = ClampMountDeg(roll_deg_ + alpha * (r_meas - roll_deg_));
        return true;
    }

    void resetQuiet() { quiet_s_ = 0.0f; }

    float pitchDeg() const { return pitch_deg_; }
    float rollDeg() const { return roll_deg_; }
    float learnedSeconds() const { return learned_s_; }

private:
    float pitch_deg_ = kDefaultHeadMountPitchDeg;
    float roll_deg_ = kDefaultHeadMountRollDeg;
    float quiet_s_ = 0.0f;
    float learned_s_ = 0.0f;
};

// Online roll/pitch gyroscope zero-rate bias tracker (gyro_x and gyro_z in imu.cc).
// Runs while the robot is quietly balancing with the head stationary; on a balanced robot standing upright,
// mean roll and pitch angular rates are identically zero, so this self-heals corrupted boot calibrations
// (e.g. when held in hand during startup) and tracks thermal drift without any yaw-spin feedback loop.
// Yaw bias (gyro_y) is intentionally kept from the static boot calibration because a balancing robot can
// slowly rotate in yaw while v_wheel == 0.
class GyroBiasEstimator {
public:
    static constexpr float kMaxBiasDps = 12.0f;
    static constexpr float kMaxRawStillDps = 18.0f;
    static constexpr float kMinQuietS = 0.6f;
    static constexpr float kFastPhaseS = 3.0f;
    static constexpr float kFastTauS = 1.2f;
    static constexpr float kSlowTauS = 8.0f;

    static float ClampBias(float dps) {
        if (dps < -kMaxBiasDps) return -kMaxBiasDps;
        if (dps > kMaxBiasDps) return kMaxBiasDps;
        return dps;
    }

    void seed(float bx, float bz) {
        if (__builtin_isfinite(bx)) bx_ = ClampBias(bx);
        if (__builtin_isfinite(bz)) bz_ = ClampBias(bz);
        seeded_ = true;
    }

    bool seeded() const { return seeded_; }

    bool update(float dt_s, bool quiet, float raw_gx, float raw_gz) {
        if (!quiet || dt_s <= 0.0f || dt_s > 0.1f ||
            !__builtin_isfinite(raw_gx) || !__builtin_isfinite(raw_gz)) {
            quiet_s_ = 0.0f;
            return false;
        }
        if (fabsf(raw_gx) > kMaxRawStillDps || fabsf(raw_gz) > kMaxRawStillDps) {
            quiet_s_ = 0.0f;
            return false;
        }
        quiet_s_ += dt_s;
        if (quiet_s_ < kMinQuietS) {
            return false;
        }
        learned_s_ += dt_s;
        const float tau = learned_s_ < kFastPhaseS ? kFastTauS : kSlowTauS;
        float alpha = dt_s / tau;
        if (alpha > 0.05f) alpha = 0.05f;
        bx_ = ClampBias(bx_ + alpha * (raw_gx - bx_));
        bz_ = ClampBias(bz_ + alpha * (raw_gz - bz_));
        return true;
    }

    void resetQuiet() { quiet_s_ = 0.0f; }
    float bx() const { return bx_; }
    float bz() const { return bz_; }
    float learnedSeconds() const { return learned_s_; }

private:
    float bx_ = 0.0f;
    float bz_ = 0.0f;
    bool seeded_ = false;
    float quiet_s_ = 0.0f;
    float learned_s_ = 0.0f;
};

}  // namespace hover_rl
