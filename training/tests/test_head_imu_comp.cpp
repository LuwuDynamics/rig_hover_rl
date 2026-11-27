// Build with c++ -std=c++17 -Imain/boards/hover from the repository root.
#include "head_imu_comp.h"
#include "rl_controller.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace hover_rl;

static constexpr float kDegToRad = 0.0174532925f;

// Synthesize imu.cc head-frame accelerometer and gyro readings for a robot with:
//   body pitch P_deg, body roll R_deg, head pan q_deg, head pan rate w_head_dps,
//   and IMU mounting tilt (pm_deg, rm_deg) inside the head.
static void synth_imu(float P_deg, float R_deg, float q_deg, float w_head_dps,
                      float pm_deg, float rm_deg,
                      float& ax, float& ay, float& az,
                      float& gx, float& gy, float& gz) {
    // Specific force in body frame (x=fwd, y=left, z=up) for pitch P (about +y) and roll R (about +x):
    //   R_wb = Ry(P) * Rx(R), world up = (0, 0, g), f_body = R_wb^T * (0, 0, g)
    const float P = P_deg * kDegToRad;
    const float R = R_deg * kDegToRad;
    const float g = 9.81f;
    // Ry(P)^T * (0, 0, g) = (-g*sin(P), 0, g*cos(P))
    // Rx(R)^T * (-g*sin(P), 0, g*cos(P)) = (-g*sin(P), g*sin(R)*cos(P), g*cos(R)*cos(P))
    // Note: in imu.cc convention roll > 0 means az > 0 => f_body_y = -g*sin(R)*cos(P)
    const float fb_x = -g * sinf(P);
    const float fb_y = -g * sinf(R) * cosf(P);
    const float fb_z = g * cosf(R) * cosf(P);

    // Body -> leveled head frame: Rz(-q)
    const float q = q_deg * kDegToRad;
    const float cq = cosf(q), sq = sinf(q);
    const float fh_x = cq * fb_x + sq * fb_y;
    const float fh_y = -sq * fb_x + cq * fb_y;
    const float fh_z = fb_z;

    // Leveled head -> tilted IMU frame: R_hi^T = Rx(+rm) * Ry(-pm)
    const float pm = pm_deg * kDegToRad;
    const float rm = rm_deg * kDegToRad;
    const float cp = cosf(pm), sp = sinf(pm);
    const float cr = cosf(rm), sr = sinf(rm);
    auto tilt = [&](float vx, float vy, float vz, float& ox, float& oy, float& oz) {
        const float x1 = cp * vx - sp * vz;
        const float y1 = vy;
        const float z1 = sp * vx + cp * vz;
        ox = x1;
        oy = cr * y1 - sr * z1;
        oz = sr * y1 + cr * z1;
    };

    float fi_x, fi_y, fi_z;
    tilt(fh_x, fh_y, fh_z, fi_x, fi_y, fi_z);
    ax = fi_z;
    ay = -fi_x;
    az = -fi_y;

    float wi_x, wi_y, wi_z;
    tilt(0.0f, 0.0f, w_head_dps, wi_x, wi_y, wi_z);
    gx = wi_x;
    gy = -wi_z;
    gz = -wi_y;
}

int main() {
    // 1. Exact 3D invariance across q_head in [-75, +75] deg and w_head = +-150 dps.
    const float pm = 5.0f, rm = -4.3f;
    for (float q : {-75.0f, -55.0f, -30.0f, 0.0f, 35.0f, 60.0f, 75.0f}) {
        float ax, ay, az, gx, gy, gz;
        synth_imu(0.0f, 0.0f, q, 150.0f, pm, rm, ax, ay, az, gx, gy, gz);
        const auto p0 = ProjectHeadImu(ax, ay, az, gx, gy, gz, q, pm, rm);
        assert(fabsf(p0.pitch_acc_deg - pm) < 1e-3f);
        assert(fabsf(p0.roll_acc_deg) < 1e-3f);
        assert(fabsf(p0.pitch_gyro_dps) < 1e-3f);

        // Non-zero body pitch (+12 deg) with zero roll: pitch and roll remain exact and orthogonal.
        synth_imu(12.0f, 0.0f, q, -150.0f, pm, rm, ax, ay, az, gx, gy, gz);
        const auto p1 = ProjectHeadImu(ax, ay, az, gx, gy, gz, q, pm, rm);
        assert(fabsf(p1.pitch_acc_deg - (12.0f + pm)) < 2e-2f);
        assert(fabsf(p1.roll_acc_deg) < 2e-2f);
        assert(fabsf(p1.pitch_gyro_dps) < 1e-3f);

        // Non-zero body roll (+8 deg) with zero pitch: does not leak into pitch.
        synth_imu(0.0f, 8.0f, q, 0.0f, pm, rm, ax, ay, az, gx, gy, gz);
        const auto p2 = ProjectHeadImu(ax, ay, az, gx, gy, gz, q, pm, rm);
        assert(fabsf(p2.pitch_acc_deg - pm) < 2e-2f);
        assert(fabsf(p2.roll_acc_deg - 8.0f) < 2e-2f);
    }

    // 2. Zero pitch shift at q_head == 0 regardless of mount_pitch_deg (no feedback loop while learning).
    {
        float ax, ay, az, gx, gy, gz;
        synth_imu(2.5f, 0.0f, 0.0f, 0.0f, 5.2f, -4.3f, ax, ay, az, gx, gy, gz);
        const float p_at_4 = ProjectHeadImu(ax, ay, az, gx, gy, gz, 0.0f, 4.0f, 0.0f).pitch_acc_deg;
        const float p_at_5 = ProjectHeadImu(ax, ay, az, gx, gy, gz, 0.0f, 5.2f, -4.3f).pitch_acc_deg;
        const float p_at_m2 = ProjectHeadImu(ax, ay, az, gx, gy, gz, 0.0f, -2.0f, 3.0f).pitch_acc_deg;
        assert(fabsf(p_at_4 - p_at_5) < 5e-2f);
        assert(fabsf(p_at_4 - p_at_m2) < 1e-1f);
    }

    // 3. Exact recovery of (pm, rm) from raw quiet accelerometer at any q_head (even -7 deg servo deadzone).
    {
        float ax, ay, az, gx, gy, gz;
        synth_imu(0.0f, 0.0f, -7.2f, 0.0f, -2.3f, 5.1f, ax, ay, az, gx, gy, gz);
        float p_out = 0.0f, r_out = 0.0f;
        assert(HeadMountFromRawAccel(ax, ay, az, &p_out, &r_out));
        assert(fabsf(p_out - (-2.3f)) < 1e-3f);
        assert(fabsf(r_out - 5.1f) < 1e-3f);
    }

    // 4. Online estimator gating, convergence, clamping, and preservation across resetQuiet().
    {
        HeadMountEstimator est;
        assert(est.pitchDeg() == kDefaultHeadMountPitchDeg);
        assert(est.rollDeg() == kDefaultHeadMountRollDeg);

        float ax, ay, az, gx, gy, gz;
        synth_imu(0.0f, 0.0f, -7.0f, 0.0f, 5.0f, -4.3f, ax, ay, az, gx, gy, gz);

        // Must not update before kMinQuietS (0.4 s) or when not quiet.
        for (int i = 0; i < 50; ++i) assert(!est.update(0.0065f, true, ax, ay, az));
        assert(est.pitchDeg() == kDefaultHeadMountPitchDeg);
        assert(!est.update(0.0065f, false, ax, ay, az));  // resets quiet streak
        for (int i = 0; i < 50; ++i) assert(!est.update(0.0065f, true, ax, ay, az));

        // Sustained quiet stand converges rapidly (<0.2 deg in ~2.6 s) and tightly after settling.
        for (int i = 0; i < 400; ++i) est.update(0.0065f, true, ax, ay, az);
        assert(fabsf(est.pitchDeg() - 5.0f) < 0.1f);
        assert(fabsf(est.rollDeg() - (-4.3f)) < 0.2f);
        for (int i = 0; i < 600; ++i) est.update(0.0065f, true, ax, ay, az);
        assert(fabsf(est.pitchDeg() - 5.0f) < 0.03f);
        assert(fabsf(est.rollDeg() - (-4.3f)) < 0.05f);

        // Rejects shock / upside-down samples and preserves learned angles across resetQuiet().
        assert(!est.update(0.0065f, true, -9.8f, 0.0f, 0.0f));
        assert(!est.update(0.0065f, true, 15.0f, 0.0f, 0.0f));
        const float saved_p = est.pitchDeg(), saved_r = est.rollDeg();
        est.resetQuiet();
        assert(est.pitchDeg() == saved_p && est.rollDeg() == saved_r);
    }

    // 5. Real rev20 quasi-static accelerometer checkpoints (from sim/docs/data/gesture_run_rev20_full_2026-11-16.json).
    {
        // Learned from 1.5 s quiet stand at qh = -7.2 deg: (pm, rm) = (+4.88, -3.70) deg.
        const float pm_real = 4.88f, rm_real = -3.70f;
        const auto p_neg55 = ProjectHeadImu(9.77f, 0.88f, -0.66f, 0, 0, 0, -54.8f, pm_real, rm_real);
        const auto p_zero = ProjectHeadImu(9.78f, 0.89f, -0.63f, 0, 0, 0, -7.1f, pm_real, rm_real);
        const auto p_pos35 = ProjectHeadImu(9.75f, 0.70f, -0.72f, 0, 0, 0, 34.0f, pm_real, rm_real);
        // Pitch stays tightly clustered around pm_real across a 90 deg head sweep; roll stays near 0.
        assert(fabsf(p_neg55.pitch_acc_deg - p_zero.pitch_acc_deg) < 0.6f);
        assert(fabsf(p_pos35.pitch_acc_deg - p_zero.pitch_acc_deg) < 1.2f);
        assert(fabsf(p_neg55.roll_acc_deg) < 1.0f);
        assert(fabsf(p_zero.roll_acc_deg) < 1.0f);
        assert(fabsf(p_pos35.roll_acc_deg) < 1.0f);
    }

    // 6. RlController applies imu_zero_deg (and gesture bias) directly without cos(q_head) attenuation.
    {
        RlController c1, c2;
        Signals s1{}, s2{};
        s1.imu_ok = s2.imu_ok = true;
        s1.imu_zero_deg = s2.imu_zero_deg = 4.0f;
        s1.pitch_deg = s2.pitch_deg = 4.0f;
        s1.q_head_deg = 0.0f;
        s2.q_head_deg = 60.0f;
        s2.yaw_deg = 60.0f;  // keep base_yaw = yaw - q_head == 0
        for (int ms = 5; ms <= 300; ms += 5) {
            s1.now_us = s1.wheel1_us = s1.wheel2_us = ms * 1000;
            s2.now_us = s2.wheel1_us = s2.wheel2_us = ms * 1000;
            const auto o1 = c1.step(s1);
            const auto o2 = c2.step(s2);
            assert(fabsf(o1.pitch_rad - o2.pitch_rad) < 1e-6f);
            assert(fabsf(o1.tor1 - o2.tor1) < 1e-4f);
            assert(fabsf(o1.tor2 - o2.tor2) < 1e-4f);
        }
    }

    // 7. RlController isolates head-spin yaw rate from policy obs/er_int while preserving body-spin response.
    {
        RlController c_head, c_body;
        Signals sh{}, sb{};
        sh.imu_ok = sb.imu_ok = true;
        sh.imu_zero_deg = sb.imu_zero_deg = 4.0f;
        sh.pitch_deg = sb.pitch_deg = 4.0f;
        // First settle 300 ms to finish ramp.
        for (int ms = 5; ms <= 300; ms += 5) {
            sh.now_us = sh.wheel1_us = sh.wheel2_us = ms * 1000;
            sb.now_us = sb.wheel1_us = sb.wheel2_us = ms * 1000;
            c_head.step(sh);
            c_body.step(sb);
        }
        // Fast +200 dps head spin with wheels stationary vs actual body spin:
        sh.head_moving = true;
        sh.yaw_rate_dps = 200.0f;
        sb.head_moving = false;
        sb.yaw_rate_dps = 200.0f;
        hover_rl::Output oh{}, ob{};
        for (int ms = 305; ms <= 560; ms += 5) {
            sh.now_us = sh.wheel1_us = sh.wheel2_us = ms * 1000;
            sb.now_us = sb.wheel1_us = sb.wheel2_us = ms * 1000;
            oh = c_head.step(sh);
            ob = c_body.step(sb);
        }
        const float cd_head = 0.5f * (oh.right - oh.left);
        const float cd_body = 0.5f * (ob.right - ob.left);
        assert(fabsf(c_head.policy().yawErrorIntegral()) < 0.05f);
        assert(fabsf(cd_head) < 5.0f);
        assert(fabsf(c_body.policy().yawErrorIntegral()) > 0.5f);
        assert(cd_body < -25.0f);

        // After head stops (even 6 deg short of target) and 0.4 s hold expires, raw gyro resumes cleanly.
        sh.head_moving = false;
        sh.yaw_rate_dps = 0.0f;
        for (int ms = 565; ms <= 1000; ms += 5) {
            sh.now_us = sh.wheel1_us = sh.wheel2_us = ms * 1000;
            c_head.step(sh);
        }
        sh.yaw_rate_dps = 120.0f;
        for (int ms = 1005; ms <= 1250; ms += 5) {
            sh.now_us = sh.wheel1_us = sh.wheel2_us = ms * 1000;
            oh = c_head.step(sh);
        }
        assert(fabsf(c_head.policy().yawErrorIntegral()) > 0.3f);
    }

    // 8. Online GyroBiasEstimator recovers true roll/pitch bias from corrupted boot calibration.
    {
        GyroBiasEstimator gbe;
        gbe.seed(0.0f, -3.25f);
        // Must not adapt during the initial 0.6 s deadtime.
        for (int i = 0; i < 80; ++i) assert(!gbe.update(0.0065f, true, 4.68f, -0.40f));
        assert(fabsf(gbe.bx() - 0.0f) < 1e-5f);
        // Quiet standing for ~4.2 s converges rapidly (e^(-2.5)*e^(-0.14) ~ 7% residual), and tightly after settling.
        for (int i = 0; i < 650; ++i) gbe.update(0.0065f, true, 4.68f, -0.40f);
        assert(fabsf(gbe.bx() - 4.68f) < 0.40f);
        assert(fabsf(gbe.bz() - (-0.40f)) < 0.25f);
        for (int i = 0; i < 1800; ++i) gbe.update(0.0065f, true, 4.68f, -0.40f);
        assert(fabsf(gbe.bx() - 4.68f) < 0.10f);
        assert(fabsf(gbe.bz() - (-0.40f)) < 0.08f);
        // Rejects fast motion spikes without corrupting learned bias.
        const float saved_x = gbe.bx();
        assert(!gbe.update(0.0065f, true, 50.0f, 0.0f));
        assert(gbe.bx() == saved_x);
    }

    std::printf("test_head_imu_comp: ALL PASSED\n");
    return 0;
}
