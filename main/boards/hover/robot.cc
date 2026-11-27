#include "robot.h"
#include "lulu_ble.h"
#include "imu.h"
#include <atomic>
#include <mutex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include "robot_action.h"
#include "application.h"
#include "board.h"
#include "display.h"
#include "settings.h"
#include "rl_controller.h"
#include "gesture_player.h"

float ctrl_vx = 0.0;
float stable_vx = 0.0;
float vx = 0.0;
float vyaw = 0.0;

float q_head=0.0;
static float target_head_pos = 0.0;
static bool s_gesture_drives_head = false;  // explicit angles take priority until the next gesture

float tor1 = 0.0;
float tor2 = 0.0;

PID pid_pos = {0,  0, 0.0,   0.000,0,	0,	0,  0,	0,	1000,	100,	 0.0001,	0,	1000,	0,	1000,	0,	1000};
PID pid_vel  = {0,  0, 0.0,   0.000,0,	0,	0,  0,	0,	100,	100,	 0.01,	0,	100,	0,	100,	0,	100};
PID pid_pit = {0,  0, 6.0,    0,    0.0, 0,	0,  0,	0,	1000,	0,	0.01,	0,	1000,	0,	1000,	0,	100};

float dq_u = 0.0;
float kd_pit = 0.7;
float dq_u_max = 60.0;

int last11Pos = 512;
int last21Pos = 512;
float wheel1_vel = 0.0;
float wheel2_vel = 0.0;
float wheek_vx = 0.0;
float wheel1_x = 0.0;
float wheel2_x = 0.0;
float wheel_x = 0.0;

float imu_zero = 0.0;
int stable_flag= 0;
float stable_pos = 0.0;
long stable_time = 0;
float stable_yaw = 0.0;
float yaw_u = 0.0;
float k_yaw = 4.0; //3.0
float yaw_ctrl_time = 0.0;

int rx_index = 0;

float lqr_k[4] = {0.0, 0.0, 0.0, 0.0};
int robot_state = 0;
static int64_t fall_since_us = 0;
static int64_t upright_since_us = 0;
static int64_t pick_up_count = 0;
// robot.cc's pick-up detection (the wheels spin much faster than the robot can drive: it was lifted): wheel_vel units, about 0.29 per mm/s, i.e. 350 = 1.2 m/s
// (measured on the robot 2026-11-16: three pushes switched the motors off with 'pick_up' at 1.19-1.25 m/s). 1.2 m/s is not 'lifted': a hard push makes the robot
// run after its own fall at that speed. In RL mode the policy glue has its own, slower-to-trigger guards (rl_controller.h: > 1.5 m/s for 0.1 s, > 0.8 m/s for 1 s,
// both wheels > 95 % for 0.5 s), so this one is only a last resort there: 520 units = 1.8 m/s. The original LQR keeps 350.
static const float kPickUpWheelVelLqr = 350.0f;
static const float kPickUpWheelVelRl = 520.0f;

static portMUX_TYPE s_unload_mux = portMUX_INITIALIZER_UNLOCKED;
static RobotUnloadInfo s_unload = {};

static void NoteUnload(const char* why, float v_wheel, float last_tor1, float last_tor2) {
    RobotUnloadInfo u = {};
    snprintf(u.why, sizeof(u.why), "%s", why);
    u.at_ms = esp_timer_get_time() / 1000;
    u.pitch_deg = pitch;
    u.roll_deg = roll;
    u.wheel_vel = 0.5f * (wheel1_vel + wheel2_vel);
    u.v_wheel = v_wheel;
    u.tor1 = last_tor1;
    u.tor2 = last_tor2;
    portENTER_CRITICAL(&s_unload_mux);
    u.count = s_unload.count + 1;
    s_unload = u;
    portEXIT_CRITICAL(&s_unload_mux);
    ESP_LOGW("ROBOT", "motors off: %s (pitch %.1f deg, roll %.1f deg, wheels %.0f units, RL speed %.2f m/s, last torque %.0f / %.0f)",
             why, u.pitch_deg, u.roll_deg, u.wheel_vel, v_wheel, last_tor1, last_tor2);
}

void RobotGetUnloadInfo(RobotUnloadInfo* out) {
    portENTER_CRITICAL(&s_unload_mux);
    *out = s_unload;
    portEXIT_CRITICAL(&s_unload_mux);
}
int control_mode = 0;  // 0=小车  1=云台
float gimbal_pitch_target = 0.0f;
float gimbal_kp = 36.0f;
float gimbal_kd = 3.6f;
float gimbal_ff = -150.0f;
static bool gimbal_pitch_latched = false;

float lp_vel = 0.7;

uint8_t isIMUInit = 0;

// 3 号舵机（头部云台）标定状态
static constexpr char kHeadZeroNamespace[] = "hover";
static constexpr char kHeadZeroKey[] = "head_zero";
static std::mutex motor_control_mutex;
static bool calibration_active = false;
std::atomic<int16_t> head_zero_pos{1500};
std::atomic<int16_t> head_fb_pos{-1};

// 11/21 号轮毂电机单独测试覆盖（只影响 sendWheelTor 中对应 ID）
volatile int16_t motor11_test_torque = 0;
volatile int16_t motor21_test_torque = 0;
volatile int64_t motor11_test_until_us = 0;
volatile int64_t motor21_test_until_us = 0;

// 首次成功联网后的保持站姿窗口（由 OnWifiConnected 触发）
static int64_t stand_hold_until_us = 0;
static bool stand_hold_reported = false;
static StaticSemaphore_t stand_event_storage;
static SemaphoreHandle_t stand_event;

void HoldStandStance(uint32_t duration_ms) {
    std::lock_guard<std::mutex> lock(motor_control_mutex);
    if (calibration_active) {
        return;
    }
    stand_hold_until_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    stand_hold_reported = false;
    ESP_LOGW("ROBOT", "hold upright stance for %u ms", (unsigned)duration_ms);
}

bool InitHeadZeroPos() {
    Settings settings(kHeadZeroNamespace, false);
    const int zero = settings.GetInt(kHeadZeroKey, -1);
    const bool calibrated = zero >= 0 && zero <= 3000;
    head_zero_pos = calibrated ? static_cast<int16_t>(zero) : 1500;
    ESP_LOGW("ROBOT", "head zero pos = %d, saved calibration = %d",
             static_cast<int>(head_zero_pos), calibrated);
    return calibrated;
}

void ClearSavedHeadZero() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(kHeadZeroNamespace, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_erase_key(handle, kHeadZeroKey);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE("ROBOT", "Failed to clear saved servo 3 center: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGW("ROBOT", "Saved servo 3 center cleared");
}

void EnterServoCalibration() {
    std::lock_guard<std::mutex> lock(motor_control_mutex);
    if (calibration_active) {
        return;
    }
    calibration_active = true;
    head_fb_pos = -1;
    motor11_test_until_us = 0;
    motor21_test_until_us = 0;
    stand_hold_until_us = 0;
    tor1 = 0.0f;
    tor2 = 0.0f;
    sendWheelTor(0, 0);
    EnableServo(3, 0);
    ESP_LOGW("ROBOT", "Calibration motor output: wheel torque zero, servo 3 torque off");
}

bool FinishServoCalibration() {
    const int16_t position = head_fb_pos;
    if (position < 0 || position > 3000) {
        ESP_LOGE("ROBOT", "invalid servo 3 position %d, calibration not saved", static_cast<int>(position));
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kHeadZeroNamespace, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_i32(handle, kHeadZeroKey, position);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE("ROBOT", "failed to save servo 3 center: %s", esp_err_to_name(err));
        return false;
    }

    std::lock_guard<std::mutex> lock(motor_control_mutex);
    head_zero_pos = position;
    target_head_pos = 0.0f;
    s_gesture_drives_head = false;
    vx = 0.0f;
    vyaw = 0.0f;
    robot_state = 0;
    stable_flag = 0;
    stable_time = esp_timer_get_time() / 1000;
    stable_yaw = yaw;
    yaw_ctrl_time = 0;
    fall_since_us = 0;
    upright_since_us = 0;
    pick_up_count = 0;
    gimbal_pitch_latched = false;
    WriteByte_P_V(3, position, 1300);
    EnableServo(3, 1);
    calibration_active = false;
    ESP_LOGW("ROBOT", "servo 3 center saved at %d", (int)head_zero_pos);
    return true;
}

void SetWheelMotorTest(uint8_t id, int speed_percent, uint32_t duration_ms) {
    if (id != 11 && id != 21) {
        ESP_LOGW("ROBOT", "invalid wheel motor id %d (only 11/21)", (int)id);
        return;
    }
    std::lock_guard<std::mutex> lock(motor_control_mutex);
    if (calibration_active) {
        ESP_LOGW("ROBOT", "Wheel test ignored during calibration");
        return;
    }
    if (speed_percent < -100) {
        speed_percent = -100;
    } else if (speed_percent > 100) {
        speed_percent = 100;
    }
    const int16_t torque = (int16_t)(speed_percent * 10);  // 100% -> 1000
    const int64_t until_us = duration_ms == 0
                                 ? 0
                                 : esp_timer_get_time() + (int64_t)duration_ms * 1000;
    if (id == 11) {
        motor11_test_torque = torque;
        motor11_test_until_us = until_us;
    } else {
        motor21_test_torque = torque;
        motor21_test_until_us = until_us;
    }
    ESP_LOGW("ROBOT", "wheel motor %d test: speed=%d%% torque=%d duration=%u ms",
             (int)id, speed_percent, (int)torque, (unsigned)duration_ms);
}

void InitializeController(){
    stand_event = xSemaphoreCreateBinaryStatic(&stand_event_storage);
    pid_pos.fpKp = 60.0;
    pid_pos.fpKi = 0.0;
    pid_pos.fpKd = 0.0;

    pid_vel.fpKp = 0.15;
    pid_vel.fpKi = 0.01;
    pid_vel.fpKd = 0.0;

    pid_pit.fpKp = 21.0;
    pid_pit.fpKi = 0.0;
    pid_pit.fpKd = 0.0;

    kd_pit = 2.3;
    // Balance point (IMU pitch at which the robot stands without torque). The old -2.0 came from holding the robot by hand, which is never exactly
    // vertical; on the real robot the RL policy settled at +3.9...+5.0 deg, so the default is centred there. Debug page variable 8 changes it live.
    imu_zero = 4.0;

    lqr_k[0] = 1300.0f;
    lqr_k[1] = 5.0f;
    lqr_k[2] = 25.0f;
    lqr_k[3] = 0.4f;

    ESP_LOGW("ROBOT_RL", "hover_rl policy built in (hover_balance_toy), default mode %d (0=LQR 1=RL 2=shadow)", (int)rl_mode);
}

void SetControlMode(int mode) {
    control_mode = (mode == 1) ? 1 : 0;
    gimbal_pitch_latched = false;
    if (control_mode == 1) {
        gimbal_pitch_target = Clip(pitch, -45.0f, 45.0f);
        ESP_LOGW("ROBOT", "mode=gimbal, pitch_target=%.1f", gimbal_pitch_target);
    } else {
        robot_state = 0;  // 回小车先倒地，避免突然平衡冲出
        ESP_LOGW("ROBOT", "mode=car");
    }
}

bool SetGimbalPitch(float angle, bool relative) {
    if (control_mode != 1) {
        return false;
    }
    // 用户语义抬头为正，IMU 抬头为负，只反目标不反力矩
    const float cmd = -angle;
    float next = relative ? (gimbal_pitch_target + cmd) : cmd;
    gimbal_pitch_target = Clip(next, -45.0f, 45.0f);
    gimbal_pitch_latched = true;
    ESP_LOGW("ROBOT", "gimbal pitch=%.1f", gimbal_pitch_target);
    return true;
}

static constexpr float kMoveLeadMaxM = 0.25f;
static constexpr float kRotateLeadMaxDeg = 200.0f;
static void RlClearBlock();      // defined with the RL controller below
static void GestureUserMotion(); // defined with the gesture player below

void RobotAddMove(float meters) {
    GestureUserMotion();
    stable_pos = Clip(stable_pos + meters, wheel_x - kMoveLeadMaxM, wheel_x + kMoveLeadMaxM);
}

void RobotAddRotate(float degrees) {
    GestureUserMotion();
    const float heading = yaw - q_head;
    stable_yaw = Clip(stable_yaw + degrees, heading - kRotateLeadMaxDeg, heading + kRotateLeadMaxDeg);
    yaw_ctrl_time = esp_timer_get_time() / 1000.0;   // only a real rotate command keeps the yaw target (see update_state)
}

void RobotStopMotion() {
    GestureUserMotion();
    stable_pos = wheel_x;
    stable_yaw = yaw - q_head;
    vx = 0.0f;
    vyaw = 0.0f;
    RlClearBlock();              // the next press of "forward" is not refused because the last one hit something
}

static void update_state(const ImuAttitude& sample) {
    static int64_t previous_sample_us = 0;
    const float pitch = sample.pitch, roll = sample.roll, yaw = sample.yaw;
    const int state_before = robot_state;
    const char* unload_why = "other";
    wheel_x = (wheel1_x+wheel2_x)/2.0/1024.0*3.1415*0.06;
    wheek_vx = (wheel1_vel + wheel2_vel)/ 2.0;
    if(fabsf(pitch) > 15.0f){
        stable_time = esp_timer_get_time()/1000.0;
    }
    if((esp_timer_get_time()/1000.0-stable_time>1000)&&stable_flag == 0){
        stable_flag = 1;
    }

    if(stable_flag == 0){
        stable_pos = wheel_x;

    }
    if((fabsf(stable_pos - wheel_x) > 0.5f || fabsf(pitch) > 30.0f) && stable_flag){
        stable_flag = 0;
    }

    // LQR: a rotate target is kept for 3 s. RL: a 360 deg spin takes ~10 s at the gentle yaw rate (0.8 rad/s), keep it for 14 s
    // (only after a rotate command has really been issued: yaw_ctrl_time is 0 until then).
    if(esp_timer_get_time()/1000.0-yaw_ctrl_time>((rl_mode == RL_MODE_RL && yaw_ctrl_time > 0.0f) ? 14000 : 3000)){
        stable_yaw = yaw - q_head;
    }

    // 姿态超出安全范围持续 → 摔倒；持续直立 → 恢复（累计时间，避免每周期刷新计时）
    static const int64_t kFallHoldUs = 500LL * 1000;
    static const int64_t kStandHoldUs = 1500LL * 1000;
    const bool fallen_pose = (fabsf(pitch) >= 60.0f || fabsf(roll) >= 40.0f);
    // Starting to balance needs the robot roughly upright for 1.5 s. A person holding it is never exactly vertical, so the RL policy (trained with random
    // initial tilts of this size) accepts +-15 deg around the balance point; the original LQR keeps +-7 deg.
    const bool rl_start = (rl_mode == RL_MODE_RL);
    const float upright_tol_deg = rl_start ? 15.0f : 7.0f;
    const bool upright_pose = (fabsf(pitch - (rl_start ? imu_zero : 0.0f)) < upright_tol_deg && fabsf(roll) < upright_tol_deg);
    const int64_t now_us = esp_timer_get_time();

    // A recovered stream can arrive between two control ticks. Its new start
    // timestamp invalidates the old upright interval even in that case.
    const bool fresh = sample.sample_us != 0 && now_us - sample.sample_us <= kImuMaxGapUs;
    const bool imu_interrupted = !fresh || previous_sample_us < sample.continuous_since_us;
    if (imu_interrupted) {
        robot_state = 0;
        unload_why = "imu";
    }
    if (imu_interrupted || sample.sample_us - previous_sample_us > kImuMaxGapUs) {
        fall_since_us = 0;
        upright_since_us = 0;
    }
    previous_sample_us = sample.sample_us;
    if (fresh && fallen_pose) {
        if (fall_since_us == 0) {
            fall_since_us = sample.sample_us;
        }
        upright_since_us = 0;
        if (sample.sample_us - fall_since_us >= kFallHoldUs) {
            robot_state = 0;
            unload_why = "fall_pose";
        }
    } else if (fresh) {
        fall_since_us = 0;

        if (upright_pose) {
            if (upright_since_us == 0) {
                upright_since_us = sample.sample_us;
            }
            if (sample.sample_us - upright_since_us >= kStandHoldUs) {
                robot_state = 1;
            }
        } else {
            upright_since_us = 0;
        }
    }

    // 首次联网提示：保持竖直平衡站姿（原地不动）15 秒，到期自动结束
    if (stand_hold_until_us != 0) {
        if (now_us >= stand_hold_until_us) {
            stand_hold_until_us = 0;
            ESP_LOGW("ROBOT", "stand hold finished");
        } else if (robot_state == 1 && upright_pose) {
            vx = 0.0f;
            vyaw = 0.0f;
            if (!stand_hold_reported) {
                stand_hold_reported = true;
                ESP_LOGW("ROBOT", "holding upright stance");
            }
        }
    }

    const float pick_up_limit = (rl_mode == RL_MODE_RL) ? kPickUpWheelVelRl : kPickUpWheelVelLqr;
    if(fabsf(wheel1_vel) > pick_up_limit||fabsf(wheel2_vel) > pick_up_limit){
        pick_up_count++;
        if(pick_up_count > 8){
            robot_state = 0;
            upright_since_us = 0;
            unload_why = "pick_up";
        }
    }else{
        pick_up_count = 0;
    }

    if (state_before == 1 && robot_state == 0) {
        NoteUnload(unload_why, 0.0f, tor1, tor2);
    }
}

// ====================================================================================================================
// Reinforcement-learning balance controller (hover_rl): plugs rl_controller.h into the existing control cycle.
//   rl_mode 0: original LQR only.
//   rl_mode 1: the policy drives the wheels. During the 0.25 s start-up ramp the LQR output is blended out (bump-less
//              hand-over); after that the policy is the only controller. A guard stop switches the motors off
//              (robot_state = 0), it never silently falls back to the LQR.
//   rl_mode 2: shadow. The LQR drives, the policy runs in the background on the same sensor data; tor values and the
//              correlation between both are published (/api/data -> "rl") to verify signs and units before closing the loop.
// ====================================================================================================================
volatile int rl_mode = HOVER_RL_DEFAULT_MODE;
volatile int rl_wheel1_is_left = 1;

static const hover_rl::Config kRlDefaults;
volatile float rl_nav_kp_yaw = kRlDefaults.nav_kp_yaw;
volatile float rl_nav_r_max = kRlDefaults.nav_r_max;
volatile float rl_nav_r_slew = kRlDefaults.nav_r_slew;
volatile float rl_nav_kp_x = kRlDefaults.nav_kp_x;
volatile float rl_nav_v_max = kRlDefaults.nav_v_max;
volatile float rl_diff_cap = kRlDefaults.diff_cap;
volatile float rl_dz_comp = kRlDefaults.dz_comp;
volatile float rl_dzlpf_ms = kRlDefaults.dz_lpf_ms;
volatile float rl_stand_sound = kRlDefaults.stand_ok_enable ? 1.0f : 0.0f;
volatile float rl_lift_v = kRlDefaults.lift_v;
volatile float rl_lift_pitch = kRlDefaults.lift_pitch_deg;
volatile float rl_lift_dq = kRlDefaults.lift_dq_dps;
volatile float rl_assist_start = kRlDefaults.assist_start_deg;
volatile float rl_assist_gain = kRlDefaults.assist_gain;
volatile float rl_wheel_max = kRlDefaults.wheel_runaway_ms;

static hover_rl::RlController s_rl;
static RlStatus s_rl_status = {};
static portMUX_TYPE s_rl_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_rl_rearm_request = false;
static unsigned s_rl_cycles = 0;
static int s_rl_stops = 0;
static int64_t s_rl_last_us = 0;
static int s_rl_stack_free = 0;
static float s_rl_agree[6] = {0, 0, 0, 0, 0, 0};   // leaky sums (lqr*rl, lqr^2, rl^2) for the forward and the yaw component
static int s_rl_prev_mode = RL_MODE_LQR;
static bool s_rl_prev_latched = false;
static unsigned s_rl_stalls = 0;
static unsigned s_rl_stands = 0;

static void RlClearBlock() {
    s_rl.clearBlock();
}

void RlGetStatus(RlStatus* out) {
    portENTER_CRITICAL(&s_rl_mux);
    *out = s_rl_status;
    portEXIT_CRITICAL(&s_rl_mux);
}

bool RobotWaitForStandEvent(uint32_t timeout_ms) {
    return xSemaphoreTake(stand_event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void RlRearm() {
    s_rl_rearm_request = true;   // executed by robot_task in the next cycle
}

const char* RlGuardName(int guard) {
    return hover_rl::GuardName((hover_rl::Guard)guard);
}

static void RlPublish(const RlStatus& st) {
    portENTER_CRITICAL(&s_rl_mux);
    s_rl_status = st;
    portEXIT_CRITICAL(&s_rl_mux);
}

static float RlAgree(float xy, float xx, float yy) {
    const float den = sqrtf(xx * yy);
    return den > 100.0f ? xy / den : 0.0f;
}

// ====================================================================================================================
// Gestures (gesture_player.h): scripted, emotion-like motions (nod, shake, greeting, ...) on top of the RL balance controller. The player runs here, in the control
// loop (GestureStep: once per cycle from RlApply, and from the gimbal branch); other tasks (the MCP tool, the debug page) only leave a request in a mailbox and wait
// for the answer. What a gesture may do: pan the head (target_head_pos), add a pitch bias to imu_zero (lean forward / back: a nod, a bow) and add rotate / move
// targets. It never touches the wheels. A cancel (guard stop, push, joystick, low battery, a new move / rotate / stop command, the request) puts the head back to
// rest and, for the safety reasons, drops the remaining move / rotate targets.
// ====================================================================================================================
volatile float rl_gesture_lean = 1.0f;
volatile float rl_head_speed = 1300.0f;
volatile float rl_gesture_quiet_s = 1.0f;

static hover_rl::GesturePlayer s_gesture;
static std::mutex s_gesture_call_mutex;                                   // one asking task at a time
// Carry motion and expression together so simultaneous callers cannot swap faces.
struct GestureRequest { int16_t motion, face; };
static std::atomic<GestureRequest> s_gesture_req{GestureRequest{}};
static int s_active_gesture_face = 0;  // owned by the control loop
static std::atomic<int> s_gesture_res{(int)hover_rl::GestureResult::kPending};
static std::atomic<int> s_gesture_cancel_req{0};                          // GestureCancel to apply in the next cycle (0 = none)
static std::atomic<uint32_t> s_gesture_face{0};                           // event for the board: bit 8 valid, bit 0 start (else end), bits 1.. gesture id
static float s_gesture_head_rest = 0.0f;                                  // target_head_pos when the gesture started
static float s_gesture_v = 0.0f;                                          // wheel speed of the last policy cycle [m/s]
static RobotGestureStatus s_gs = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0u, 0u, -1, 0};              // control-loop private (last_result -1 = never asked)
static RobotGestureStatus s_gesture_status = {0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0u, 0u, -1, 0};  // published copy
static portMUX_TYPE s_gesture_mux = portMUX_INITIALIZER_UNLOCKED;

static void GestureUserMotion() {      // a user move / rotate / stop command: whatever plays gives way
    s_gesture_cancel_req.store((int)hover_rl::GestureCancel::kUserMotion);
}

bool RobotSetHeadAngle(float degrees) {
    std::lock_guard<std::mutex> lock(motor_control_mutex);
    if (calibration_active) return false;
    target_head_pos = Clip(degrees, -75.0f, 75.0f);
    s_gesture_drives_head = false;
    return true;
}

static void GestureFace(int id, bool start) {
    if (id == kGestureMotionOnly) {
        return;
    }
    s_gesture_face.store(0x100u | ((uint32_t)id << 1) | (start ? 1u : 0u));
}

// One control cycle of the gesture player. Returns the pitch bias [deg] to add to imu_zero (0 when nothing plays).
static float GestureStep(int64_t now_us, bool rl_balancing) {
    using namespace hover_rl;
    GestureCtx ctx;
    ctx.now_us = now_us;
    ctx.balancing = rl_balancing;
    ctx.lean_deg = pitch - imu_zero;        // physical lean, without the gesture's own bias
    ctx.dq_dps = dq;
    ctx.v_wheel = s_gesture_v;
    ctx.joystick = fabsf(vx) > 0.5f || fabsf(vyaw) > 0.5f;
    const BatteryVoltageSample batt = GetBatteryVoltageSample();
    ctx.battery_v = (batt.received_us != 0 && now_us - batt.received_us < 10000000) ? batt.volts : 0.0f;
    s_gesture.cfg.lean_scale = rl_gesture_lean;
    s_gesture.cfg.start_quiet_s = rl_gesture_quiet_s;

    const int why = s_gesture_cancel_req.exchange(0);
    if (why != 0) {
        s_gesture.cancel((GestureCancel)why);
    }
    const GestureRequest req = s_gesture_req.exchange(GestureRequest{});
    if (req.motion != 0) {
        const GestureResult res = s_gesture.request((GestureId)req.motion, ctx);
        if (res == GestureResult::kOk) {
            s_active_gesture_face = req.face;
            s_gesture_head_rest = target_head_pos;
            s_gesture_drives_head = true;
        }
        s_gs.last_result = (int)res;
        s_gesture_res.store((int)res);
    }
    const GestureOut g = s_gesture.step(ctx);

    float bias = 0.0f;
    if (g.id != GestureId::kNone) {
        const GestureInfo& info = GestureInfoOf(g.id);
        if (g.started) {
            s_gs.plays++;
            GestureFace(s_active_gesture_face, true);
            ESP_LOGI("ROBOT_GESTURE", "start %s (%s), lean scale %.2f", info.name, info.text, (double)rl_gesture_lean);
        }
        for (int i = 0; i < g.n_events; i++) {
            if (g.ev[i].kind == GestureEvent::kRotate) {          // the same raw "+= degrees" as the stock double tap (no lead limit)
                stable_yaw += g.ev[i].value;
                yaw_ctrl_time = esp_timer_get_time() / 1000.0;
            } else if (g.ev[i].kind == GestureEvent::kMove) {
                stable_pos = Clip(stable_pos + g.ev[i].value, wheel_x - kMoveLeadMaxM, wheel_x + kMoveLeadMaxM);
            }
        }
        if (g.head_on && s_gesture_drives_head) {
            target_head_pos = Clip(s_gesture_head_rest + g.head_deg, -75.0f, 75.0f);
            s_gs.head_deg = g.head_deg;
        } else {
            s_gs.head_deg = 0.0f;
        }
        if (g.finished) {
            GestureFace(s_active_gesture_face, false);
            if (g.cancelled != GestureCancel::kNone) {
                s_gs.cancels++;
                s_gs.last_cancel = (int)g.cancelled;
                if (GestureNeedsBalance(g.id) && g.cancelled != GestureCancel::kJoystick && g.cancelled != GestureCancel::kUserMotion) {
                    stable_pos = wheel_x;               // drop what is left of the gesture's move / rotate targets (the joystick / the user's own command keeps its say)
                    stable_yaw = yaw - q_head;
                }
                ESP_LOGW("ROBOT_GESTURE", "%s cancelled after %.1f s: %s (lean %.1f deg, speed %.2f m/s, battery %.1f V)", info.name, (double)g.t_s,
                         GestureCancelName(g.cancelled), (double)ctx.lean_deg, (double)ctx.v_wheel, (double)ctx.battery_v);
            } else {
                ESP_LOGI("ROBOT_GESTURE", "%s done", info.name);
            }
        }
        s_gs.id = (int)g.id;
        s_gs.t_s = g.t_s;
        s_gs.dur_s = info.dur_s;
        bias = g.active ? g.bias_deg : 0.0f;
    }
    s_gs.active = g.active ? 1 : 0;
    s_gs.quiet_s = s_gesture.quietFor();
    s_gs.bias_deg = bias;
    if (!g.active) {
        s_gs.head_deg = 0.0f;
    }
    portENTER_CRITICAL(&s_gesture_mux);
    s_gesture_status = s_gs;
    portEXIT_CRITICAL(&s_gesture_mux);
    return bias;
}

int RobotGesturePlay(int id, int face_id) {
    const int count = static_cast<int>(hover_rl::GestureId::kCount);
    if (id <= 0 || id >= count || face_id < kGestureMotionOnly || face_id >= count) {
        return static_cast<int>(hover_rl::GestureResult::kUnknown);
    }
    std::lock_guard<std::mutex> lock(s_gesture_call_mutex);
    s_gesture_res.store((int)hover_rl::GestureResult::kPending);
    s_gesture_req.store({static_cast<int16_t>(id), static_cast<int16_t>(face_id ? face_id : id)});
    for (int i = 0; i < 30; i++) {          // the control loop answers within a few ms; 30 x 10 ms is the patience
        vTaskDelay(pdMS_TO_TICKS(10));
        const int r = s_gesture_res.load();
        if (r != (int)hover_rl::GestureResult::kPending) {
            return r;
        }
    }
    s_gesture_req.store(GestureRequest{});
    return (int)hover_rl::GestureResult::kPending;
}

void RobotGestureCancel() {
    s_gesture_cancel_req.store((int)hover_rl::GestureCancel::kRequested);
}

bool RobotTakeGestureFace(int* id, bool* start) {
    const uint32_t v = s_gesture_face.exchange(0);
    if (!(v & 0x100u)) {
        return false;
    }
    *id = (int)((v >> 1) & 0x7Fu);
    *start = (v & 1u) != 0;
    return true;
}

void RobotGetGestureStatus(RobotGestureStatus* out) {
    portENTER_CRITICAL(&s_gesture_mux);
    *out = s_gesture_status;
    portEXIT_CRITICAL(&s_gesture_mux);
}

// Called once per control cycle right after the LQR has computed tor1/tor2 (robot_state is already evaluated).
static void RlApply(const ImuAttitude& sample) {
    const int64_t now_us = esp_timer_get_time();
    if (s_rl_rearm_request) {
        s_rl_rearm_request = false;
        s_rl.rearm();
        s_rl_prev_latched = false;
        ESP_LOGW("ROBOT_RL", "RL controller re-armed");
    }
    static unsigned s_stack_probe = 0;
    if ((s_stack_probe++ & 0xFF) == 0) {
        s_rl_stack_free = (int)uxTaskGetStackHighWaterMark(nullptr);
    }
    const int mode = rl_mode;
    const float lqr1 = tor1, lqr2 = tor2;   // what the original controller wants

    RlStatus st = {};
    st.mode = mode;
    st.latched = s_rl.latched() ? 1 : 0;
    st.stops = s_rl_stops;
    st.cycles = s_rl_cycles;
    st.stack_free = s_rl_stack_free;
    st.dt_ms = s_rl_last_us ? (float)(now_us - s_rl_last_us) * 1e-3f : 0.0f;
    s_rl_last_us = now_us;
    st.lqr_tor1 = st.out_tor1 = lqr1;
    st.lqr_tor2 = st.out_tor2 = lqr2;
    st.lead_m = stable_pos - wheel_x;
    st.stalls = s_rl_stalls;
    st.stands = s_rl_stands;

    const bool balancing = (robot_state == 1) && isIMUInit && control_mode == 0;
    if ((mode != RL_MODE_RL && mode != RL_MODE_SHADOW) || !balancing) {
        GestureStep(now_us, false);      // head-only gestures also run while carried
        if (mode == RL_MODE_RL) {
            // Not balancing (lying, held in a hand, lifted, after a guard stop): the position and heading targets follow the robot, so that it stands
            // where it is put down instead of walking back to where it was picked up (the wheels spin when it is lifted, wheel_x jumps, and the old
            // stable_pos stays valid as long as the pitch stays within 30 deg), or turning back to an old rotate target (kept 14 s in RL mode).
            stable_pos = wheel_x;
            stable_yaw = yaw - q_head;
            yaw_ctrl_time = 0.0f;
        }
        if (s_rl_status.active) xSemaphoreTake(stand_event, 0);
        s_rl.reset();       // keep the estimators clean: the next start is a fresh start
        s_rl_prev_mode = mode;
        RlPublish(st);
        return;
    }
    if (mode != s_rl_prev_mode) {
        if (mode == RL_MODE_RL) {
            s_rl.reset();   // shadow/LQR -> RL: restart the 0.25 s hand-over ramp
        }
        s_rl_prev_mode = mode;
    }

    s_rl.cfg.wheel1_is_left = (rl_wheel1_is_left != 0);
    s_rl.cfg.nav_kp_yaw = rl_nav_kp_yaw;
    s_rl.cfg.nav_r_max = rl_nav_r_max;
    s_rl.cfg.nav_r_slew = rl_nav_r_slew;
    s_rl.cfg.nav_kp_x = rl_nav_kp_x;
    s_rl.cfg.nav_v_max = rl_nav_v_max;
    s_rl.cfg.diff_cap = rl_diff_cap;
    s_rl.cfg.dz_comp = rl_dz_comp;
    s_rl.cfg.dz_lpf_ms = rl_dzlpf_ms;
    s_rl.cfg.stand_ok_enable = (rl_stand_sound >= 0.5f);
    s_rl.cfg.lift_v = rl_lift_v;
    s_rl.cfg.lift_pitch_deg = rl_lift_pitch;
    s_rl.cfg.lift_dq_dps = rl_lift_dq;
    s_rl.cfg.assist_start_deg = rl_assist_start;
    s_rl.cfg.assist_gain = rl_assist_gain;
    s_rl.cfg.wheel_runaway_ms = rl_wheel_max;
    float w1x = 0, w2x = 0;
    int64_t w1us = 0, w2us = 0;
    GetWheelSnapshot(&w1x, &w2x, &w1us, &w2us);

    const float gesture_bias = GestureStep(now_us, mode == RL_MODE_RL);   // pitch bias of the gesture that is playing (0 = none)
    hover_rl::Signals sig;
    sig.now_us = now_us;
    sig.imu_ok = true;
    sig.pitch_deg = sample.pitch;
    sig.dq_dps = sample.pitch_rate_dps;
    sig.yaw_rate_dps = sample.yaw_rate_dps;
    sig.yaw_deg = sample.yaw;
    sig.q_head_deg = q_head;
    sig.imu_zero_deg = imu_zero + gesture_bias;
    sig.wheel1_counts = w1x;
    sig.wheel2_counts = w2x;
    sig.wheel1_us = w1us;
    sig.wheel2_us = w2us;
    sig.wheel_x_m = wheel_x;
    sig.stable_pos_m = stable_pos;
    sig.stable_yaw_deg = stable_yaw;
    sig.joy_vx = vx;
    sig.joy_vyaw = vyaw;
    const hover_rl::Output o = s_rl.step(sig);
    s_gesture_v = o.v_wheel;
    s_rl_cycles++;
    if (o.gave_up) {
        s_rl_stalls++;
        st.stalls = s_rl_stalls;
        ESP_LOGW("ROBOT_RL", "blocked: the robot pushes against something without moving, move target dropped for a while (v %.3f m/s, pitch %.1f deg)",
                 o.v_wheel, o.pitch_rad * 57.29578f);
    }

    if (o.stood) {
        s_rl_stands++;
        st.stands = s_rl_stands;
        xSemaphoreGive(stand_event);
        ESP_LOGI("ROBOT_RL", "standing: steady for a moment after the start (pitch %.1f deg, speed %.3f m/s)", o.pitch_rad * 57.29578f, o.v_wheel);
    }
    static int64_t quiet_since_us = 0;
    static int64_t last_observed_us = 0;
    const bool quiet = mode == RL_MODE_RL && o.run && !o.stop && !o.latched && o.ramp >= 1.0f &&
        !s_gs.active && fabsf(sig.q_head_deg) < 5.0f && sig.joy_vx == 0 && sig.joy_vyaw == 0 &&
        fabsf(o.v_cmd) < 0.025f && fabsf(o.r_cmd) < 0.12f && fabsf(o.v_wheel) < 0.06f &&
        fabsf(sig.dq_dps) < 15.0f && fabsf(sig.yaw_rate_dps) < 15.0f;
    if (!quiet) quiet_since_us = 0;
    else if (!quiet_since_us || now_us - last_observed_us > 50000 || o.stood) quiet_since_us = now_us;
    last_observed_us = now_us;
    st.quiet_since_us = quiet_since_us;
    st.measured_pitch_deg = sig.pitch_deg;
    st.active = 1;
    st.guard = (int)o.guard;
    st.latched = o.latched ? 1 : 0;
    st.pitch_deg = o.pitch_rad * 57.29578f;
    st.pitch_rate_dps = o.pitch_rate * 57.29578f;
    st.yaw_rate_dps = o.yaw_rate * 57.29578f;
    st.v_wheel = o.v_wheel;
    st.v_cmd = o.v_cmd;
    st.r_cmd = o.r_cmd;
    st.ramp = o.ramp;
    if (o.run) {
        st.rl_tor1 = o.tor1;
        st.rl_tor2 = o.tor2;
    }
    if (o.latched && !s_rl_prev_latched) {
        ESP_LOGE("ROBOT_RL", "RL latched after repeated yaw runaway: check wheel mapping (debug var RL_w1_left) or call set_controller rearm");
    }
    s_rl_prev_latched = o.latched;

    // agreement between the two controllers (shadow mode): forward demand f = (tor1 - tor2) / 2, yaw demand y = (tor1 + tor2) / 2
    if (mode == RL_MODE_SHADOW && o.run) {
        const float k = 1.0f - 0.0065f / 2.0f;   // ~2 s window
        const float fl = 0.5f * (lqr1 - lqr2), fr = 0.5f * (o.tor1 - o.tor2);
        const float yl = 0.5f * (lqr1 + lqr2), yr = 0.5f * (o.tor1 + o.tor2);
        s_rl_agree[0] = k * s_rl_agree[0] + fl * fr;
        s_rl_agree[1] = k * s_rl_agree[1] + fl * fl;
        s_rl_agree[2] = k * s_rl_agree[2] + fr * fr;
        s_rl_agree[3] = k * s_rl_agree[3] + yl * yr;
        s_rl_agree[4] = k * s_rl_agree[4] + yl * yl;
        s_rl_agree[5] = k * s_rl_agree[5] + yr * yr;
    }
    st.agree_fwd = RlAgree(s_rl_agree[0], s_rl_agree[1], s_rl_agree[2]);
    st.agree_yaw = RlAgree(s_rl_agree[3], s_rl_agree[4], s_rl_agree[5]);

    if (mode == RL_MODE_RL) {
        if (o.reanchor) {            // joystick (vx / vyaw) in use: hold wherever the robot ends up
            stable_pos = wheel_x;
            stable_yaw = yaw - q_head;
        }
        if (o.cancel_move) {         // blocked: give the move up (the heading target stays)
            stable_pos = wheel_x;
        }
        if (o.stop) {
            xSemaphoreTake(stand_event, 0);
            s_rl_stops++;
            NoteUnload(hover_rl::GuardName(o.guard), o.v_wheel, s_rl_status.out_tor1, s_rl_status.out_tor2);
            robot_state = 0;         // motors off; the robot has to stand upright for 1.5 s again before it restarts
            upright_since_us = 0;
            tor1 = 0.0f;
            tor2 = 0.0f;
            ESP_LOGW("ROBOT_RL", "guard stop: %s (pitch %.1f deg, yaw rate %.1f deg/s, speed %.2f m/s)",
                     hover_rl::GuardName(o.guard), st.pitch_deg, st.yaw_rate_dps, o.v_wheel);
        } else if (o.latched) {
            tor1 = 0.0f;
            tor2 = 0.0f;
        } else if (o.run) {
            tor1 = Clip(o.tor1 + (1.0f - o.ramp) * lqr1, -1000.0f, 1000.0f);
            tor2 = Clip(o.tor2 + (1.0f - o.ramp) * lqr2, -1000.0f, 1000.0f);
        }
        // else: first tick after a (re)start / gap / wheels not refreshed yet -> the LQR output stays for this cycle
        st.out_tor1 = tor1;
        st.out_tor2 = tor2;
    }
    RlPublish(st);
}

static void UpdateWheelTorque() {
    float temp_u = 0.0;
    float lqr_x = 0.0;
    float lqr_vx = 0.0;
    float lqr_q = 0.0;
    float lqr_dq = 0.0;
    const auto sample = imu_get_attitude();
    const float pitch = sample.pitch, dq = sample.pitch_rate_dps, yaw = sample.yaw;
    update_state(sample);
    if (control_mode == 1) {
        GestureStep(esp_timer_get_time(), false);   // gimbal mode also supports head-only gestures
    }

    if (control_mode == 1) {
        // 云台：右轮 PD 跟俯仰，左轮 8 档旋钮，头部照旧
        if (isIMUInit && !gimbal_pitch_latched) {
            gimbal_pitch_target = Clip(pitch, -45.0f, 45.0f);
            gimbal_pitch_latched = true;
        }
        if (isIMUInit) {
            const float des = Clip(gimbal_pitch_target, -45.0f, 45.0f);
            const float pd = gimbal_kp * (des - pitch) - gimbal_kd * dq;
            const float ff = gimbal_ff * sinf(pitch * PI / 180.0f);
            tor2 = Clip(pd + ff, -500.0f, 500.0f);
        } else {
            tor2 = 0;
        }

        const float step = 1024.0f / 8.0f;
        float pos = (float)last11Pos;
        float target = roundf(pos / step) * step;
        if (target >= 1024.0f) target -= 1024.0f;
        if (target < 0.0f) target += 1024.0f;
        float err = target - pos;
        if (err > 512.0f) err -= 1024.0f;
        if (err < -512.0f) err += 1024.0f;
        if (fabsf(err) < 6.0f) err = 0.0f;  // 到位死区，避免在档位上抖动
        tor1 = Clip(1.5f * err, -80.0f, 80.0f);
    } else {

    // pid_pos.fpDes = stable_pos;
    // pid_pos.fpFB = wheel_x;
    // CalIWeakenPID(&pid_pos);

    // pid_vel.fpDes = vx + pid_pos.fpU;
    // pid_vel.fpFB = wheek_vx;
    // CalIWeakenPID(&pid_vel);

    // pid_pit.fpDes = imu_zero*cos(q_head*PI/180.0)+ pid_vel.fpU;
    // pid_pit.fpFB = pitch;       //调节平衡角度
    // CalIWeakenPID(&pid_pit);

    // dq_u = -kd_pit*dq;
    // dq_u = Clip(dq_u, -dq_u_max, dq_u_max);
    // if(pid_pit.fpU<0&&pid_pit.fpU>-5){
    //     pid_pit.fpU = -5;
    // }else if(pid_pit.fpU>0&&pid_pit.fpU<5){
    //     pid_pit.fpU = 5;
    // }

    // if(pid_pit.fpU<-20){
    //     pid_pit.fpU = pid_pit.fpU -10;
    // }else if(pid_pit.fpU>20){
    //     pid_pit.fpU = pid_pit.fpU +10;
    // }
    // temp_u = pid_pit.fpU + dq_u;

        {
            // 离散 LQR 全状态反馈: u = -K·x, Ts≈6ms (robot_control 周期)
            // x1 位置误差(m)  x2 速度误差(m/s)  x3 俯仰误差(deg)  x4 俯仰角速度(deg/s)
            // pitch_ref = imu_zero·cos(head) 补偿头部姿态对平衡角的影响
            const float pitch_ref = imu_zero * cosf(q_head * PI / 180.0f);
            lqr_x  = wheel_x - stable_pos;
            lqr_vx = wheek_vx - vx;
            lqr_q  = pitch - pitch_ref;
            lqr_dq = dq;

            temp_u = -(lqr_k[0] * lqr_x
                      + lqr_k[1] * lqr_vx
                      + lqr_k[2] * lqr_q
                      + lqr_k[3] * lqr_dq);

            if (temp_u < -20.0f) {
                temp_u -= 10.0f;
            } else if (temp_u > 20.0f) {
                temp_u += 10.0f;
            }
            temp_u = Clip(temp_u, -pid_pit.fpUMax, pid_pit.fpUMax);
        }

        yaw_u = k_yaw*(yaw-q_head-stable_yaw);
        yaw_u = Clip(yaw_u, -80, 80);
        if(robot_state==0){
            tor1 = 0;
            tor2 = 0;
        }else{
            tor1 = -temp_u + yaw_u;
            tor2 = temp_u + yaw_u;
        }
        RlApply(sample);   // hover_rl: replaces tor1/tor2 in mode 1, only observes them in mode 2
    }
}

void robot_control() {
    static int64_t next_voltage_poll_us = esp_timer_get_time() + kBatteryVoltagePollIntervalUs;
    {
        std::lock_guard<std::mutex> lock(motor_control_mutex);
        if (calibration_active) {
            EnableServo(3, 0);
        } else {
            UpdateWheelTorque();
            WriteByte_P_V(3, short(head_zero_pos - target_head_pos*10.0), (short)rl_head_speed);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    {
        std::lock_guard<std::mutex> lock(motor_control_mutex);
        if (!calibration_active && (control_mode == 1 || isIMUInit == 1)) {
            sendWheelTor(short(tor1), short(tor2));
        } else {
            sendWheelTor(0, 0);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    {
        std::lock_guard<std::mutex> lock(motor_control_mutex);
        // Keep polling the head position so the touch can save the adjusted center.
        if (rx_index == 0) {
            ReadWheelState(1);
        } else if (rx_index == 1) {
            ReadWheelState(2);
        } else if (rx_index == 2) {
            ReadWheelState(11);
        } else if (rx_index == 3) {
            ReadWheelState(21);
        } else if (rx_index == 5 && esp_timer_get_time() >= next_voltage_poll_us) {
            // Use the second head-servo slot so the voltage read adds no bus transaction.
            ReadServoVoltage(3);
            next_voltage_poll_us = esp_timer_get_time() + kBatteryVoltagePollIntervalUs;
        } else {
            ReadMotorState(3);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    if (rx_index++ > 4) {
        rx_index = 0;
    }
}

// 将 0~255 映射到 [min, max]
static int from_order_range(uint8_t b, int min, int max) {
    return min + (int)b * (max - min) / 255;
}

// LULU BLE 遥控协议解析入口
// data: 一次 GATT 写入的完整帧（APP 侧按遥控协议打包）
void lulu_ble_on_rx_bytes(const uint8_t* data, size_t len) {
    if (!data || len < 7) {
        return;
    }

    // 查找帧头 55 00
    size_t i = 0;
    while (i + 1 < len && !(data[i] == 0x55 && data[i + 1] == 0x00)) {
        ++i;
    }
    if (i + 1 >= len) {
        return;
    }

    const uint8_t* frame = &data[i];
    size_t remaining = len - i;
    if (remaining < 7) {
        return;
    }

    uint8_t length = frame[2];
    if (length > remaining) {
        // 不完整帧，丢弃
        return;
    }

    uint8_t order = frame[3];
    const uint8_t* payload = &frame[4];
    size_t payload_len = length - 7;
    if (4 + payload_len + 3 > remaining) {
        return;
    }

    uint8_t checksum = frame[4 + payload_len];
    uint8_t tail0    = frame[4 + payload_len + 1];
    uint8_t tail1    = frame[4 + payload_len + 2];

    if (tail0 != 0x00 || tail1 != 0xAA) {
        return;
    }

    // 校验和：LENGTH + ORDER + PAYLOAD 所有字节
    uint32_t sum = length + order;
    for (size_t j = 0; j < payload_len; ++j) {
        sum += payload[j];
    }
    sum &= 0xFF;
    if (checksum != (uint8_t)(0xFF - sum)) {
        return;
    }

    // 处理读命令 (ORDER_READ = 0x02)
    if (order == 0x02) {
        if (payload_len < 1) return;
        uint8_t addr = payload[0];
        ESP_LOGI("ROBOT_BLE", "Read command: addr=0x%02X", addr);

        // 构建响应帧: 55 00 LENGTH READ_READBACK(0x12) ADDR DATA... CHECKSUM 00 AA
        uint8_t resp[32];
        size_t resp_len = 0;

        if (addr == 0x07) {
            // versionNumber: 返回版本字符串，如 "L-1.0.0"
            const char* version = "L-1.0.0";
            size_t ver_len = strlen(version);

            // 帧格式: 55 00 LENGTH ORDER ADDR DATA... CHECKSUM 00 AA
            // LENGTH = 整帧长度 = 2 + 1 + 1 + 1 + ver_len + 1 + 2 = 8 + ver_len
            resp[0] = 0x55;
            resp[1] = 0x00;
            resp[2] = 8 + ver_len;  // LENGTH = 整帧长度
            resp[3] = 0x12;         // READ_READBACK
            resp[4] = addr;
            memcpy(&resp[5], version, ver_len);

            // 计算校验和
            uint32_t s = resp[2] + resp[3] + resp[4];
            for (size_t j = 0; j < ver_len; j++) s += resp[5 + j];
            resp[5 + ver_len] = (uint8_t)(0xFF - (s & 0xFF));
            resp[6 + ver_len] = 0x00;
            resp[7 + ver_len] = 0xAA;
            resp_len = 8 + ver_len;
        } else if (addr == 0x01) {
            // battery: 返回电池电量 (0-100)
            resp[0] = 0x55;
            resp[1] = 0x00;
            resp[2] = 8;    // LENGTH
            resp[3] = 0x12; // READ_READBACK
            resp[4] = addr;
            resp[5] = 100;  // 电池电量 100%
            uint32_t s = resp[2] + resp[3] + resp[4] + resp[5];
            resp[6] = (uint8_t)(0xFF - (s & 0xFF));
            resp[7] = 0x00;
            resp[8] = 0xAA;
            resp_len = 9;
        } else {
            // 其他地址返回 0
            resp[0] = 0x55;
            resp[1] = 0x00;
            resp[2] = 8;
            resp[3] = 0x12;
            resp[4] = addr;
            resp[5] = 0x00;
            uint32_t s = resp[2] + resp[3] + resp[4] + resp[5];
            resp[6] = (uint8_t)(0xFF - (s & 0xFF));
            resp[7] = 0x00;
            resp[8] = 0xAA;
            resp_len = 9;
        }

        if (resp_len > 0) {
            ESP_LOGI("ROBOT_BLE", "Sending response: len=%d, data=%02X %02X %02X %02X %02X...",
                     resp_len, resp[0], resp[1], resp[2], resp[3], resp[4]);
            lulu_ble_send(resp, resp_len);
        }
        return;
    }

    // 处理写命令 (0x00/0x01)
    if (order != 0x00 && order != 0x01) {
        return;
    }

    if (payload_len < 2) {
        return;
    }

    uint8_t addr  = payload[0];
    uint8_t value = payload[1];

    switch (addr) {
    case 0x30: { // speedVx: 前后速度
        int v = from_order_range(value, -100, 100);
        vx = (float)v;
        break;
    }
    case 0x32: { // speedVyaw: 原地转向速度
        int w = from_order_range(value, -100, 100);
        vyaw = (float)w;
        break;
    }
    default:
        // 其他地址暂不处理，保留给后续扩展
        break;
    }
}

float Clip(float fpValue, float fpMin, float fpMax)
{
	if(fpValue <= fpMin)
	{
		return fpMin;
	}
	else if(fpValue >= fpMax)
	{
		return fpMax;
	}
	else
	{
		return fpValue;
	}
}
