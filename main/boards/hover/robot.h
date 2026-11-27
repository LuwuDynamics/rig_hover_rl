#ifndef __ROBOT_H
#define __ROBOT_H
#include "motor.h"
#include <stddef.h>
#include <atomic>

#define FLASH_ZERO_POS_ADDR 0xFFF000
#define PI 3.14159

typedef struct
{
    float fpDes;
    float fpFB;
    float fpKp;
    float fpKi;
    float fpKd;
    float fpE;
    float fpPreE;
    float fpSumE;
    float fpU;
    float fpUMax;
    float fpEMax;
    float fpEMin;
    float fpUp;
    float fpPMax;
    float fpUi;
    float fpIMax;
    float fpUd;
    float fpDMax;
} PID;

void robot_control();
float Clip(float fpValue, float fpMin, float fpMax);
void lulu_ble_on_rx_bytes(const uint8_t* data, size_t len);
void InitializeController();
void SetControlMode(int mode);
bool SetGimbalPitch(float angle, bool relative);
bool RobotSetHeadAngle(float degrees);   // absolute pan angle; returns false during servo calibration
// 在指定时长内保持竖直平衡站姿（原地不动），用于首次联网成功后的 15 秒站稳提示
void HoldStandStance(uint32_t duration_ms);

// 3 号舵机（头部云台）标定
bool InitHeadZeroPos();          // 是否读到有效的已保存中位
void ClearSavedHeadZero();      // 删除已保存的舵机中位
void EnterServoCalibration();    // 轮毂零扭矩，头部舵机卸力
bool FinishServoCalibration();   // 保存当前位置为中位并恢复力矩

// 11/21 号轮毂电机独立测试控制（speed_percent: -100..100，duration_ms=0 停止）
void SetWheelMotorTest(uint8_t id, int speed_percent, uint32_t duration_ms);
extern volatile int16_t motor11_test_torque;
extern volatile int16_t motor21_test_torque;
extern volatile int64_t motor11_test_until_us;
extern volatile int64_t motor21_test_until_us;

extern std::atomic<int16_t> head_zero_pos;
extern std::atomic<int16_t> head_fb_pos;

extern int control_mode;
extern float gimbal_pitch_target;
extern float gimbal_kp;
extern float gimbal_kd;
extern float gimbal_ff;
extern float vx;
extern float vyaw;
extern float q_head;
extern float dq_u_max;
extern uint8_t isIMUInit;
extern float imu_zero;
extern float stable_pos;
extern float stable_yaw;
extern float k_yaw;
extern float kd_pit;
extern float lp_vel;
extern float yaw_ctrl_time;
extern PID pid_pos;
extern PID pid_vel;
extern PID pid_pit;
extern float lqr_k[4];

extern float wheel1_vel;
extern float wheel2_vel;
extern float wheel1_x;
extern float wheel2_x;
extern float wheel_x;            // mean odometry [m], D = 60 mm, forward positive
extern int last11Pos;
extern int last21Pos;
extern float tor1;
extern float tor2;
extern int rx_index;

// ---------------------------------------------------------------------------------------------------------------
// Reinforcement-learning balance controller (hover_rl, see rl_controller.h / hover_policy.h / README_RL.md)
//   rl_mode 0: original LQR only
//   rl_mode 1: the RL policy drives the wheels (LQR is only blended in during the 0.25 s start-up ramp)
//   rl_mode 2: shadow - the LQR drives, the policy is computed in the background and logged for comparison
// ---------------------------------------------------------------------------------------------------------------
#ifndef HOVER_RL_DEFAULT_MODE
#define HOVER_RL_DEFAULT_MODE 1
#endif
enum { RL_MODE_LQR = 0, RL_MODE_RL = 1, RL_MODE_SHADOW = 2 };
extern volatile int rl_mode;
extern volatile int rl_wheel1_is_left;   // 1: tor1 is the policy's "left" wheel (default). If the robot turns the wrong way set 0 (no re-flash needed)

// Live tuning of the RL command shaping (debug page vars 20..25, applied every control cycle; defaults = hover_rl::Config in rl_controller.h)
extern volatile float rl_nav_kp_yaw;     // rotate: heading P gain [1/s]
extern volatile float rl_nav_r_max;      // rotate: maximum commanded yaw rate [rad/s]
extern volatile float rl_nav_r_slew;     // rotate: slew limit of the commanded yaw rate [rad/s^2], 0 = off
extern volatile float rl_nav_kp_x;       // move: position P gain [1/s]
extern volatile float rl_nav_v_max;      // move: maximum commanded speed [m/s]
extern volatile float rl_dz_comp;        // dead-zone compensation added to the wheel torque in its direction [command units], 0 = off
extern volatile float rl_diff_cap;       // cap of the differential wheel torque [command units], 0 = off (smaller = gentler but may not turn)
extern volatile float rl_dzlpf_ms;       // low-pass of the input of the dead-zone compensation [ms], 0 = off (breaks the 15-25 Hz limit cycle the compensation can start)
extern volatile float rl_lift_pitch;     // lift detection: the body counts as 'hardly moving' below this |pitch| [deg] (default 10)
extern volatile float rl_lift_dq;        // ... and below this |pitch rate| [deg/s] (default 60)
extern volatile float rl_lift_v;         // lift detection: wheels faster than this while the body hardly moves (|pitch| < 6 deg, |pitch rate| < 50 deg/s) for 0.1 s = lifted, motors off [m/s], 0 = off
extern volatile float rl_assist_start;   // catch assist for hard pushes: lean (pitch + 0.1 s * pitch rate) beyond which the wheels get at least rl_assist_gain * excess [deg], 0 = off
extern volatile float rl_assist_gain;    // [command units per deg of excess lean]
extern volatile float rl_wheel_max;      // wheel-speed guard: faster than this for 0.1 s switches the motors off [m/s] (a slower, longer limit of 0.8 m/s for 1 s is fixed)
extern volatile float rl_stand_sound;    // play the "you can let go" prompt when the robot has stood up and is steady (rl_controller.h Config::stand_ok_*): 1 = on, 0 = off

typedef struct {
    int mode;                      // configured mode
    int active;                    // 1 while the policy ran in this cycle (mode 1/2 and robot balancing)
    int guard;                     // last guard (see RlGuardName)
    int latched;                   // yaw-runaway latch: output stays 0 until RlRearm()
    int stops;                     // number of guard stops since boot
    float pitch_deg, pitch_rate_dps, yaw_rate_dps;   // what the policy saw (pitch relative to imu_zero)
    float v_wheel, v_cmd, r_cmd;   // m/s, m/s, rad/s
    float ramp;                    // start-up ramp 0..1
    float rl_tor1, rl_tor2;        // what the policy wants on the bus
    float lqr_tor1, lqr_tor2;      // what the original LQR wants on the bus
    float out_tor1, out_tor2;      // what is actually sent
    float agree_fwd, agree_yaw;    // shadow mode: correlation of LQR and policy forward / yaw demand over ~2 s (-1..1; 0 = no excitation)
    float dt_ms;                   // control period
    int stack_free;                // robot_task minimum free stack (bytes)
    unsigned cycles;               // control cycles in which the policy ran
    float lead_m;                  // stable_pos - wheel_x: how far the move target is ahead of the robot [m]
    int64_t quiet_since_us;         // uninterrupted quiet RL control, for stand-angle measurements
    float measured_pitch_deg;       // absolute body pitch, before subtracting the balance reference
    unsigned stalls;               // moves given up because the robot was pushing against something
    unsigned stands;               // number of times the robot stood up and was steady ("you can let go") since boot
} RlStatus;
void RlGetStatus(RlStatus* out);

void RlRearm();                    // clear the yaw-runaway latch (takes effect in the next control cycle)
const char* RlGuardName(int guard);

// Motion targets (move / rotate / stop). Used by the MCP tools and the debug page so that every path behaves the same:
//  * a target never runs further ahead of the robot than 25 cm / 200 deg (the phone remote sends a step every 0.8 s while a key is held,
//    the robot is slower than that, so the targets used to pile up and the robot kept walking long after the key was released),
//  * RobotStopMotion() drops whatever is left: the robot stops where it is.
// Why the motors were last switched off while the robot was balancing (robot_state 1 -> 0). `why` is "fall_pose" (pitch >= 60 deg or roll >= 40 deg for 0.5 s),
// "pick_up" (wheels faster than the pick-up limit), "imu", or the name of the RL guard ("wheel_runaway", "saturated", ...). Shown on the debug page
// (/api/data -> "unload") and in the serial log ("motors off: ..."). /api/trace still holds the ~2.6 s of control loop right before it.
typedef struct {
    unsigned count;                // number of such switch-offs since boot
    char why[16];
    int64_t at_ms;                 // esp_timer time of the last one [ms]
    float pitch_deg, roll_deg;     // IMU pitch / roll at that moment
    float wheel_vel;               // firmware wheel speed units (about 0.25 per mm/s), mean of both wheels
    float v_wheel;                 // m/s as the RL controller saw it (0 when the switch-off did not come from it)
    float tor1, tor2;              // the torque commands that were last sent
} RobotUnloadInfo;
void RobotGetUnloadInfo(RobotUnloadInfo* out);

bool RobotWaitForStandEvent(uint32_t timeout_ms);  // Wait for a standing cue or the next periodic prompt-task update.
void RobotAddMove(float meters);
void RobotAddRotate(float degrees);
void RobotStopMotion();

// ---------------------------------------------------------------------------------------------------------------
// Gestures: scripted, emotion-like motions (nod, shake, greeting, ...) on top of the RL balance controller, see gesture_player.h.
// The player runs inside the control loop; these calls only leave a request / read the state, so they are safe from any task.
// Head shakes and left/right looks use the head servo in any balance state. Gestures that lean, move or turn the body
// require a quiet stand and stop on loss of balance.
// ---------------------------------------------------------------------------------------------------------------
extern volatile float rl_gesture_lean;     // multiplier of the lean (pitch bias) amplitude of every gesture, 0.2 .. 1.5 (debug page var RL_gesture_lean)
extern volatile float rl_gesture_quiet_s;  // time standing quietly before a body gesture can start [s]
extern volatile float rl_head_speed;       // the speed argument of the head servo command (WriteByte_P_V, stock 1300; unit not documented: debug page var 38 HEAD_speed lets you try others, the head speed shows in /api/trace qhead_cdeg)
// A zero face_id uses the gesture's own expression. Motion-only gestures emit no display events.
constexpr int kGestureMotionOnly = -1;
int RobotGesturePlay(int id, int face_id = 0);              // GestureId (1..11); returns a GestureResult (0 = started); waits up to 0.3 s for the control loop's decision
void RobotGestureCancel();                 // stop what is playing: the head goes back to rest, the remaining move / rotate targets are dropped
bool RobotTakeGestureFace(int* id, bool* start);   // for the board: true once per start / end of a gesture, to show / clear its emotion on the screen
typedef struct {
    int active;                    // a gesture is playing
    int id;                        // GestureId of the one playing, else of the last one played (0 = none yet)
    float t_s, dur_s;
    float bias_deg;                // what is added to imu_zero right now
    float head_deg;                // offset of the head target from its rest angle
    unsigned plays;                // gestures started since boot
    unsigned cancels;              // ... that were cancelled before the end
    int last_result;               // GestureResult of the last request
    int last_cancel;               // GestureCancel of the last cancelled gesture
    float quiet_s;                 // how long the robot has stood quietly (a gesture needs rl_gesture_quiet_s)
} RobotGestureStatus;
void RobotGetGestureStatus(RobotGestureStatus* out);

#endif
