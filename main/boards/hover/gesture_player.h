// Gesture player for the RIG-Hover desk toy: scripted, emotion-like motions that ride on top of the RL balance controller.
//
// Header-only, no dependencies beyond <math.h> / <stdint.h>. The same file is compiled into the firmware (boards/hover/robot.cc) and into the simulator
// bridge (native/fw_bridge.cpp -> tools/gesture_study.py, tests/test_gestures.py), so what the simulator validates is what runs on the robot.
//
// A gesture only uses channels the firmware already has; it never touches the wheels directly:
//   head    pan angle of the head servo [deg, + = left] relative to where the head was when the gesture started (independent of the balance loop)
//   bias    pitch bias added to imu_zero [deg, + = lean forward]: the policy tries to hold pitch_rel = 0, so a pulse of the bias = the body leans forward / back and
//           comes back (a nod, a bow, a rock). The robot walks a few cm while it does that and the position hold brings it home.
//   rotate / move   one-shot body turn [deg] / step [m] through the move / rotate targets (the outer loop of the glue, closed loop on the odometry)
//   emotion the face shown on the screen while it plays (the board maps the name to its emoji)
// Why not the joystick channel: the policy slews it at 0.3 m/s^2 / 4 rad/s^2, a square wave on the yaw command comes out 2-3 times too big with 5-15 cm of drift
// (docs/INTEGRATION.md 10.17); the lean pulses and the rotate targets are what the simulator found well behaved.
//
// Safety: a body gesture starts only after the robot has stood quietly for a moment, and it cancels itself (bias back to 0 at once, head back to rest) when the robot stops
// balancing (a guard stop, picked up, fallen), when the lean or the wheel speed leave the range of the gestures (pushed), when the phone joystick is used,
// when the battery is low, or on request (new move / rotate / stop command, the user, the MCP tool).
#pragma once
#include <math.h>
#include <stdint.h>

namespace hover_rl {

enum class GestureId : int { kNone = 0, kHello, kWave, kYes, kNo, kLook, kHappy, kStartle, kShy, kSleepy, kBow, kSpin, kCount };

enum class GestureResult : int { kOk = 0, kUnknown, kBusy, kNotBalancing, kNotQuiet, kLowBattery, kCooldown, kPending = -1 };

enum class GestureCancel : int { kNone = 0, kNotBalancing, kLean, kSpeed, kJoystick, kBattery, kRequested, kUserMotion };

inline bool GestureNeedsBalance(GestureId id) {
    return id != GestureId::kNo && id != GestureId::kLook;
}

inline const char* GestureResultName(GestureResult r) {
    switch (r) {
        case GestureResult::kOk: return "ok";
        case GestureResult::kUnknown: return "unknown";
        case GestureResult::kBusy: return "busy";
        case GestureResult::kNotBalancing: return "not_balancing";
        case GestureResult::kNotQuiet: return "not_quiet";
        case GestureResult::kLowBattery: return "low_battery";
        case GestureResult::kCooldown: return "cooldown";
        default: return "pending";
    }
}

inline const char* GestureCancelName(GestureCancel c) {
    switch (c) {
        case GestureCancel::kNone: return "-";
        case GestureCancel::kNotBalancing: return "not_balancing";
        case GestureCancel::kLean: return "lean";
        case GestureCancel::kSpeed: return "speed";
        case GestureCancel::kJoystick: return "joystick";
        case GestureCancel::kBattery: return "battery";
        case GestureCancel::kRequested: return "requested";
        case GestureCancel::kUserMotion: return "user_motion";
    }
    return "?";
}

struct GestureInfo {
    const char* name;      // short English name, what the MCP tool / the tools take
    const char* emotion;   // emoji of the board's display (boards/hover/emoji)
    float dur_s;
    const char* text;      // one line for the MCP tool description / the debug page
};

inline const GestureInfo& GestureInfoOf(GestureId id) {
    static const GestureInfo kTable[] = {
        {"none", "neutral", 0.0f, ""},
        {"hello", "happy", 8.0f, "turn 30 deg, wave the head, bow, turn back"},
        {"wave", "winking", 5.0f, "wave the head and bow, no body turn"},
        {"yes", "happy", 3.0f, "nod twice"},
        {"no", "confused", 3.0f, "shake the head"},
        {"look", "thinking", 4.2f, "look left, then right, then back"},
        {"happy", "laughing", 3.0f, "bob and wag the head (excited)"},
        {"startle", "surprised", 3.4f, "snap the head away and jump back"},
        {"shy", "embarrassed", 8.0f, "look away, turn away and back off a little, then return"},
        {"sleepy", "sleepy", 5.0f, "slowly nod off, then wake up with a start"},
        {"bow", "relaxed", 3.2f, "bow"},
        {"spin", "silly", 12.0f, "one full turn on the spot"},
    };
    int i = (int)id;
    if (i < 0 || i >= (int)GestureId::kCount) i = 0;
    return kTable[i];
}

inline GestureId GestureFromName(const char* s) {
    if (!s) return GestureId::kNone;
    for (int i = 1; i < (int)GestureId::kCount; i++) {
        const char* a = GestureInfoOf((GestureId)i).name;
        const char* b = s;
        while (*a && *a == *b) { a++; b++; }
        if (*a == 0 && *b == 0) return (GestureId)i;
    }
    return GestureId::kNone;
}

struct GestureConfig {
    float lean_scale = 1.0f;          // multiplies every bias (lean) amplitude; the debug page var RL_gesture_lean
    float max_bias_deg = 8.0f;        // hard limit of |bias| after the scaling
    float max_head_deg = 75.0f;       // limit of the absolute head target (servo range of the MCP tool)
    // start conditions: the robot has to stand quietly for a moment (judged relative to its own standing pitch, so a robot whose imu_zero is a few degrees off still starts)
    float start_quiet_s = 1.0f;
    float quiet_lean_dev_deg = 2.5f;  // the lean may not deviate more than this from its own 1.5 s average
    float quiet_lean_abs_deg = 8.0f;  // ... and may not be further than this from imu_zero
    float quiet_v = 0.06f;            // 0.3 s average of |wheel speed| [m/s]
    float quiet_dq = 40.0f;           // 0.3 s average of |pitch rate| [deg/s] (the standing dither of the policy reaches 35 deg/s in the simulator)
    float gap_s = 0.5f;               // minimum time between two gestures
    float start_battery_v = 7.2f;     // below this no gesture starts (0.1 V steps, 0 = unknown = allowed)
    // cancel conditions (the normal gestures stay below 20 deg and 0.35 m/s)
    float cancel_lean_deg = 24.0f;    // physical lean (pitch - imu_zero) beyond this: pushed
    float cancel_v = 0.5f;            // wheel speed beyond this [m/s]: pushed / running away
    float cancel_battery_v = 6.9f;
};

struct GestureCtx {
    int64_t now_us = 0;
    bool balancing = false;           // the RL policy drives the wheels (robot_state 1, car mode, RL mode, no guard stop)
    float lean_deg = 0.0f;            // physical pitch minus imu_zero, WITHOUT the gesture's own bias [deg]
    float dq_dps = 0.0f;
    float v_wheel = 0.0f;             // [m/s]
    bool joystick = false;            // the phone joystick (vx / vyaw) is in use
    float battery_v = 0.0f;           // 0 = unknown
};

struct GestureEvent {
    enum Kind : uint8_t { kNone = 0, kRotate, kMove };
    uint8_t kind = kNone;
    float value = 0.0f;               // deg (rotate, + = left) or m (move, + = forward)
};

struct GestureOut {
    bool active = false;              // a gesture is playing in this step (apply head / bias)
    bool started = false;             // first step of a gesture: show the emotion
    bool finished = false;            // the step in which it ended, normally or cancelled: bias is 0, the head goes back to rest
    GestureCancel cancelled = GestureCancel::kNone;
    GestureId id = GestureId::kNone;
    float t_s = 0.0f;
    bool head_on = false;             // the gesture drives the head in this step
    float head_deg = 0.0f;            // offset from the rest angle [deg]
    float bias_deg = 0.0f;            // add to imu_zero [deg]
    int n_events = 0;
    GestureEvent ev[4];
};

class GesturePlayer {
public:
    GestureConfig cfg;

    // ---- shape of the gestures (all times in seconds; the numbers are what tools/gesture_study.py validated in the simulator) ----
    static float Lin(float t, const float* pts, int n) {            // piecewise linear through (t0, v0, t1, v1, ...), held after the last point
        if (t <= pts[0]) return pts[1];
        for (int i = 1; i < n; i++) {
            if (t < pts[2 * i]) {
                const float f = (t - pts[2 * i - 2]) / (pts[2 * i] - pts[2 * i - 2]);
                return pts[2 * i - 1] + f * (pts[2 * i + 1] - pts[2 * i - 1]);
            }
        }
        return pts[2 * n - 1];
    }
    static float Bump(float t, float t0, float width, float amp) {  // one smooth pulse: amp * sin^2
        if (t < t0 || t >= t0 + width) return 0.0f;
        const float s = sinf(3.14159265f * (t - t0) / width);
        return amp * s * s;
    }
    static float Burst(float t, float f, int n, float amp) {        // n whole cycles of a sine wave with a soft start and end
        const float T = (float)n / f;
        if (t < 0.0f || t >= T) return 0.0f;
        return amp * sinf(6.28318531f * f * t) * sqrtf(sinf(3.14159265f * t / T));
    }

    // Evaluate the shape of gesture `id` at time t: the head offset [deg] (has_head = the gesture drives the head) and the unscaled pitch bias [deg].
    static void Shape(GestureId id, float t, float* head, bool* has_head, float* bias) {
        *head = 0.0f;
        *has_head = false;
        *bias = 0.0f;
        switch (id) {
            case GestureId::kHello:
                *head = Burst(t - 1.8f, 2.0f, 3, 28.0f);
                *has_head = true;
                *bias = Bump(t, 3.7f, 1.2f, 4.0f);
                break;
            case GestureId::kWave:
                *head = Burst(t - 0.3f, 2.0f, 3, 28.0f);
                *has_head = true;
                *bias = Bump(t, 2.2f, 1.2f, 4.0f);
                break;
            case GestureId::kYes:
                *bias = Bump(t, 0.2f, 0.8f, 4.0f) + Bump(t, 1.2f, 0.8f, 4.0f);
                break;
            case GestureId::kNo:
                *head = Burst(t, 1.2f, 3, 32.0f);
                *has_head = true;
                break;
            case GestureId::kLook: {
                static const float k[] = {0.0f, 0.0f, 0.5f, 45.0f, 1.4f, 45.0f, 2.2f, -45.0f, 3.1f, -45.0f, 3.7f, 0.0f};
                *head = Lin(t, k, 6);
                *has_head = true;
                break;
            }
            case GestureId::kHappy:
                *head = Burst(t, 2.0f, 5, 20.0f);
                *has_head = true;
                *bias = Burst(t, 2.0f, 5, 3.0f);      // 4 deg excites the 2 Hz resonance of some plants beyond 20 deg
                break;
            case GestureId::kStartle: {
                static const float k[] = {0.0f, 0.0f, 0.15f, -55.0f, 2.0f, -55.0f, 3.0f, 0.0f};
                *head = Lin(t, k, 4);
                *has_head = true;
                *bias = Bump(t, 0.05f, 0.5f, -6.0f);
                break;
            }
            case GestureId::kShy: {
                static const float k[] = {0.0f, 0.0f, 1.0f, -60.0f, 5.0f, -60.0f, 6.0f, 0.0f};
                *head = Lin(t, k, 4);
                *has_head = true;
                break;
            }
            case GestureId::kSleepy: {
                static const float kb[] = {0.0f, 0.0f, 3.0f, 3.0f, 3.15f, -3.0f, 3.6f, 0.0f};
                static const float kh[] = {0.0f, 0.0f, 3.0f, -20.0f, 3.2f, 0.0f};
                *bias = Lin(t, kb, 4);
                *head = Lin(t, kh, 3);
                *has_head = true;
                break;
            }
            case GestureId::kBow:
                *bias = Bump(t, 0.3f, 1.4f, 5.0f);
                break;
            default:
                break;
        }
    }

    // One-shot body events of gesture `id`: (time, kind, value); returns the number of events, sorted by time.
    static int Events(GestureId id, float* t, uint8_t* kind, float* value) {
        int n = 0;
        auto add = [&](float tt, uint8_t k, float v) { t[n] = tt; kind[n] = k; value[n] = v; n++; };
        switch (id) {
            case GestureId::kHello:
                add(0.0f, GestureEvent::kRotate, 30.0f);
                add(5.3f, GestureEvent::kRotate, -30.0f);
                break;
            case GestureId::kShy:
                add(0.5f, GestureEvent::kRotate, -30.0f);
                add(1.0f, GestureEvent::kMove, -0.04f);
                add(4.5f, GestureEvent::kRotate, 30.0f);
                add(5.0f, GestureEvent::kMove, 0.04f);
                break;
            case GestureId::kSpin:      // the same raw "+= 360" as the stock double tap (the firmware does not clamp the lead for gesture events)
                add(0.0f, GestureEvent::kRotate, 360.0f);
                break;
            default:
                break;
        }
        return n;
    }

    // ---- control ----
    // Ask for a gesture. Call from the control loop (the same thread as step()). The start conditions are checked with the context of this cycle.
    GestureResult request(GestureId id, const GestureCtx& c) {
        if (id <= GestureId::kNone || id >= GestureId::kCount) return GestureResult::kUnknown;
        if (id_ != GestureId::kNone) return GestureResult::kBusy;
        if (GestureNeedsBalance(id) && !c.balancing) return GestureResult::kNotBalancing;
        if (c.battery_v > 0.0f && c.battery_v < cfg.start_battery_v) return GestureResult::kLowBattery;
        if (end_us_ != 0 && (float)(c.now_us - end_us_) * 1e-6f < cfg.gap_s) return GestureResult::kCooldown;
        if (GestureNeedsBalance(id) && quiet_s_ < cfg.start_quiet_s) return GestureResult::kNotQuiet;
        id_ = id;
        start_us_ = c.now_us;
        next_ev_ = 0;
        n_ev_ = Events(id, ev_t_, ev_kind_, ev_val_);
        start_pending_ = true;
        cancel_req_ = GestureCancel::kNone;
        plays_++;
        return GestureResult::kOk;
    }

    // Cancel at the next step (also from the same thread only; other threads go through the robot.cc mailbox).
    void cancel(GestureCancel why = GestureCancel::kRequested) { if (id_ != GestureId::kNone) cancel_req_ = why; }

    // One control cycle (call every cycle, also while no gesture is playing: it tracks how long the robot has stood quietly).
    GestureOut step(const GestureCtx& c) {
        GestureOut o;
        const float dt = last_us_ ? clampf((float)(c.now_us - last_us_) * 1e-6f, 0.0f, 0.1f) : 0.0f;
        last_us_ = c.now_us;
        if (!c.balancing) {
            ema_init_ = false;
            quiet_s_ = 0.0f;
        } else if (!ema_init_) {
            ema_init_ = true;
            lean_ref_ = c.lean_deg;
            v_ema_ = fabsf(c.v_wheel);
            dq_ema_ = fabsf(c.dq_dps);
            quiet_s_ = 0.0f;
        } else {
            lean_ref_ += (c.lean_deg - lean_ref_) * clampf(dt / 1.5f, 0.0f, 1.0f);
            v_ema_ += (fabsf(c.v_wheel) - v_ema_) * clampf(dt / 0.3f, 0.0f, 1.0f);
            dq_ema_ += (fabsf(c.dq_dps) - dq_ema_) * clampf(dt / 0.3f, 0.0f, 1.0f);
            const bool quiet = fabsf(c.lean_deg - lean_ref_) < cfg.quiet_lean_dev_deg && fabsf(c.lean_deg) < cfg.quiet_lean_abs_deg &&
                               v_ema_ < cfg.quiet_v && dq_ema_ < cfg.quiet_dq && !c.joystick;
            quiet_s_ = quiet ? quiet_s_ + dt : 0.0f;
        }
        if (id_ == GestureId::kNone) return o;

        const GestureInfo& info = GestureInfoOf(id_);
        const float t = (float)(c.now_us - start_us_) * 1e-6f;
        GestureCancel why = cancel_req_;
        cancel_req_ = GestureCancel::kNone;
        if (why == GestureCancel::kNone && GestureNeedsBalance(id_)) {
            if (!c.balancing) why = GestureCancel::kNotBalancing;
            else if (fabsf(c.lean_deg) > cfg.cancel_lean_deg) why = GestureCancel::kLean;
            else if (fabsf(c.v_wheel) > cfg.cancel_v) why = GestureCancel::kSpeed;
        }
        if (why == GestureCancel::kNone) {
            if (c.joystick) why = GestureCancel::kJoystick;
            else if (c.battery_v > 0.0f && c.battery_v < cfg.cancel_battery_v) why = GestureCancel::kBattery;
        }
        o.id = id_;
        o.t_s = t;
        o.started = start_pending_;
        start_pending_ = false;
        if (why != GestureCancel::kNone || t >= info.dur_s) {
            o.finished = true;
            o.cancelled = why;
            o.head_on = true;          // back to rest
            last_cancel_ = why;
            if (why != GestureCancel::kNone) cancels_++;
            end_us_ = c.now_us;
            id_ = GestureId::kNone;
            return o;
        }
        o.active = true;
        float head, bias;
        bool has_head;
        Shape(id_, t, &head, &has_head, &bias);
        o.head_on = has_head;
        o.head_deg = clampf(head, -cfg.max_head_deg, cfg.max_head_deg);
        o.bias_deg = clampf(bias * clampf(cfg.lean_scale, 0.2f, 1.5f), -cfg.max_bias_deg, cfg.max_bias_deg);
        while (next_ev_ < n_ev_ && t >= ev_t_[next_ev_] && o.n_events < 4) {
            o.ev[o.n_events].kind = ev_kind_[next_ev_];
            o.ev[o.n_events].value = ev_val_[next_ev_];
            o.n_events++;
            next_ev_++;
        }
        return o;
    }

    bool active() const { return id_ != GestureId::kNone; }
    GestureId id() const { return id_; }
    float quietFor() const { return quiet_s_; }
    unsigned plays() const { return plays_; }
    unsigned cancels() const { return cancels_; }
    GestureCancel lastCancel() const { return last_cancel_; }
    void reset() {     // forget the running gesture and the quiet timer; the configuration stays
        const GestureConfig keep = cfg;
        *this = GesturePlayer();
        cfg = keep;
    }

private:
    static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    GestureId id_ = GestureId::kNone;
    int64_t start_us_ = 0, end_us_ = 0, last_us_ = 0;
    float quiet_s_ = 0.0f, lean_ref_ = 0.0f, v_ema_ = 0.0f, dq_ema_ = 0.0f;
    bool ema_init_ = false;
    bool start_pending_ = false;
    GestureCancel cancel_req_ = GestureCancel::kNone, last_cancel_ = GestureCancel::kNone;
    unsigned plays_ = 0, cancels_ = 0;
    int next_ev_ = 0, n_ev_ = 0;
    float ev_t_[6] = {}, ev_val_[6] = {};
    uint8_t ev_kind_[6] = {};
};

}  // namespace hover_rl
