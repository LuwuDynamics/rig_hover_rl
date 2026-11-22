// hover_policy.h -- RL balance / drive / turn policy for RIG-Hover (header-only, no heap, float32).
//
// Generated weights live in hover_policy_weights.h (tools/export_policy.py). This header is the contract:
// it turns firmware-side sensor values + user commands into the two wheel commands, exactly as the simulator
// (hover_sim/env.py) feeds the network during training.  Verified against the simulator by tests/test_hover_policy.cpp.
//
// Frames and signs (PHYSICAL, not register signs):
//   forward = the way the screen faces, pitch > 0 = leaning forward, yaw rate > 0 = turning left (CCW from above)
//   left/right command > 0 = that wheel drives the robot forward.
//   The firmware maps them to the bus: wheel 1 (tor1) positive = forward, wheel 2 (tor2) negative = forward
//   (robot.cc: tor1 = -u, tor2 = +u for a forward lean); which of the two is the left wheel must be checked on the robot.
#pragma once
#include <math.h>
#include <stdint.h>
#include "hover_policy_weights.h"

namespace hover_rl {

struct Inputs {
    float pitch_rad;          // measured pitch minus the balance offset (imu_zero), forward lean positive
    float pitch_rate_rad_s;   // d(pitch)/dt (the firmware's `dq`)
    float yaw_rate_rad_s;     // bias-compensated gyro about the vertical axis, left turn positive
    float wheel_speed_m_s;    // R * (w_left + w_right) / 2, wheel speed RELATIVE TO THE BODY (encoder), forward positive
    float cmd_speed_m_s;      // user target speed (clamped to +-0.15)
    float cmd_yaw_rate_rad_s; // user target yaw rate (clamped to +-2)
    float dt_s;               // time since the previous call (control period, 4..9 ms)
};

struct Outputs {
    float left;               // wheel command units, +forward, within +-1000
    float right;
    float act_common;         // raw network outputs after clipping to +-1
    float act_diff;
};

class Policy {
public:
    static constexpr float kV_ACCEL = 0.3f;       // m/s^2 command slew
    static constexpr float kR_ACCEL = 4.0f;       // rad/s^2
    static constexpr float kV_MAX = 0.15f;        // m/s, user speed command limit (desktop toy)
    static constexpr float kR_MAX = 2.0f;         // rad/s, user yaw-rate command limit
    static constexpr float kA_LAT_MAX = 1.0f;     // m/s^2, |v*r| cap on the commanded yaw rate (static tipping limit ~4)
    static constexpr float kEV_LEAK_S = 120.0f, kEV_CLIP = 0.25f;
    static constexpr float kER_LEAK_S = 30.0f, kER_CLIP = 1.0f;
    static constexpr float kEP_LEAK_S = 60.0f, kEP_CLIP = 0.4f;   // integral of the position error (overcomes wheel friction)
    static constexpr float kCMD_UNITS = 1000.0f;

    void reset() { v_cmd_ = r_cmd_ = ev_int_ = er_int_ = ep_int_ = 0.0f; last_ = Outputs{0, 0, 0, 0}; }

    // One control tick.
    Outputs step(const Inputs& in) {
        const float dt = in.dt_s;
        // 1) command shaping (slew + lateral-acceleration cap)
        const float v_t = clampf(in.cmd_speed_m_s, -kV_MAX, kV_MAX);
        v_cmd_ += clampf(v_t - v_cmd_, -kV_ACCEL * dt, kV_ACCEL * dt);
        const float r_cap = fminf(kR_MAX, kA_LAT_MAX / fmaxf(fabsf(v_cmd_), 0.12f));
        const float r_t = clampf(in.cmd_yaw_rate_rad_s, -r_cap, r_cap);
        r_cmd_ += clampf(r_t - r_cmd_, -kR_ACCEL * dt, kR_ACCEL * dt);
        // 2) leaky error integrals
        ev_int_ += (in.wheel_speed_m_s - v_cmd_) * dt;
        ev_int_ -= ev_int_ * dt / kEV_LEAK_S;
        ev_int_ = clampf(ev_int_, -kEV_CLIP, kEV_CLIP);
        ep_int_ += ev_int_ * dt;
        ep_int_ -= ep_int_ * dt / kEP_LEAK_S;
        ep_int_ = clampf(ep_int_, -kEP_CLIP, kEP_CLIP);
        er_int_ += (in.yaw_rate_rad_s - r_cmd_) * dt;
        er_int_ -= er_int_ * dt / kER_LEAK_S;
        er_int_ = clampf(er_int_, -kER_CLIP, kER_CLIP);
        // 3) network
        float obs[kHoverPolicyObs] = {in.pitch_rad, in.pitch_rate_rad_s, in.yaw_rate_rad_s, in.wheel_speed_m_s,
                                      v_cmd_, r_cmd_, ev_int_, er_int_, ep_int_};
        float h1[kHoverPolicyH1], h2[kHoverPolicyH2], a[kHoverPolicyAct];
        for (int i = 0; i < kHoverPolicyObs; i++) obs[i] = clampf(obs[i] * kHoverPolicyObsScale[i], -5.0f, 5.0f);
        for (int j = 0; j < kHoverPolicyH1; j++) {
            float s = kHoverPolicyB1[j];
            for (int i = 0; i < kHoverPolicyObs; i++) s += kHoverPolicyW1[j][i] * obs[i];
            h1[j] = tanhf(s);
        }
        for (int j = 0; j < kHoverPolicyH2; j++) {
            float s = kHoverPolicyB2[j];
            for (int i = 0; i < kHoverPolicyH1; i++) s += kHoverPolicyW2[j][i] * h1[i];
            h2[j] = tanhf(s);
        }
        for (int j = 0; j < kHoverPolicyAct; j++) {
            float s = kHoverPolicyB3[j];
            for (int i = 0; i < kHoverPolicyH2; i++) s += kHoverPolicyW3[j][i] * h2[i];
            a[j] = clampf(s, -1.0f, 1.0f);
        }
        // 4) wheel mixing: left = common - diff, right = common + diff (diff > 0 turns left)
        last_.act_common = a[0];
        last_.act_diff = a[1];
        last_.left = clampf(a[0] - a[1], -1.0f, 1.0f) * kCMD_UNITS;
        last_.right = clampf(a[0] + a[1], -1.0f, 1.0f) * kCMD_UNITS;
        return last_;
    }

    float speedCommand() const { return v_cmd_; }
    float yawRateCommand() const { return r_cmd_; }
    float speedErrorIntegral() const { return ev_int_; }
    float positionErrorIntegral() const { return ep_int_; }
    // Clear the speed-error and position-error integrals (they are inputs of the network, so 0 is always a valid state):
    // used when the robot is blocked and keeps pushing, so the push does not wind up.
    void clearPositionIntegrals() { ev_int_ = 0.0f; ep_int_ = 0.0f; }
    float yawErrorIntegral() const { return er_int_; }
    const Outputs& last() const { return last_; }

private:
    static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    float v_cmd_ = 0, r_cmd_ = 0, ev_int_ = 0, er_int_ = 0, ep_int_ = 0;
    Outputs last_{0, 0, 0, 0};
};

// Optional position / heading outer loop for the existing MCP semantics ("move N cm", "rotate N degrees", "spin 360").
// P controllers on firmware-side measurements (wheel odometry in metres, gyro-integrated yaw in radians, left positive),
// producing the speed / yaw-rate commands that Policy::step() expects.  Same constants as tools/nav_eval.py.
class Navigator {
public:
    static constexpr float kKpX = 1.5f, kVMax = 0.10f;       // 1/s, m/s   (the validated config of tools/nav_eval.py; the policy clamps to 0.15 anyway)
    static constexpr float kKpYaw = 2.0f, kRMax = 1.5f;      // 1/s, rad/s

    void reset(float x_m, float yaw_rad) { x_t_ = x_m; yaw_t_ = yaw_rad; }
    void move(float distance_m) { x_t_ += distance_m; }          // + forward
    void rotate(float angle_rad) { yaw_t_ += angle_rad; }        // + left (counter-clockwise)
    void update(float x_m, float yaw_rad, float* cmd_speed, float* cmd_yaw_rate) const {
        *cmd_speed = clampf(kKpX * (x_t_ - x_m), -kVMax, kVMax);
        *cmd_yaw_rate = clampf(kKpYaw * (yaw_t_ - yaw_rad), -kRMax, kRMax);
    }
private:
    static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    float x_t_ = 0, yaw_t_ = 0;
};

}  // namespace hover_rl
