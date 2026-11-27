// rl_controller.h -- RIG-Hover 强化学习平衡控制器的固件侧胶水层
//
// header-only，不依赖 ESP-IDF（只用 <math.h>/<stdint.h>），所以可以在主机上和仿真一起编译测试
// （sim/tools/fw_bridge_eval.py 用的就是这份代码）。
//
// 它把固件里已有的量（imu.cc 的 pitch/dq/偏航角速度、motor.cc 的轮子编码器、robot.cc 的 stable_pos/stable_yaw）
// 整理成 hover_policy.h 要求的输入，调用策略，再把结果映射回 tor1/tor2，并带一组保护。
//
// 符号约定（全部从固件现有的、已经能站稳的 LQR 代码推出，不依赖“物理上的左右”）：
//   * “前进为正” = 现有 LQR 用来纠正“俯仰为正”的那个方向：轮 1（tor1）正为前进，轮 2（tor2）负为前进；
//     wheel1_x / wheel2_x 已经是“前进为正”的累计编码器计数（motor.cc 里轮 2 取了反）。
//   * 偏航：imu.cc 的 yaw 变量增大 = 策略眼里的“左转”（MCP rotate “正为左”也是这个方向）。
//     现有偏航回路 tor1 = ... + yaw_u, tor2 = ... + yaw_u，yaw_u = k_yaw * (yaw - q_head - stable_yaw) 是负反馈，
//     由此可知：让 yaw 增大的转动 = 朝轮 1 一侧转 => 对策略来说轮 1 是“左轮”（cfg.wheel1_is_left = true）。
//   * 速度：用编码器位置的差分（每个轮子在自己的刷新时刻差分，再做 EMA），不依赖速度寄存器的比例。
#pragma once
#include <atomic>
#include <math.h>
#include <stdint.h>

#include "hover_policy.h"

namespace hover_rl {

// 保护触发原因（kImu、kGap 只是本周期不输出，其余都会请求停机）
enum class Guard : uint8_t {
    kNone = 0,
    kImu,           // IMU 没就绪或数值异常
    kGap,           // 两次调用间隔 > 40 ms：本周期不输出，估计器重新起步
    kTilt,          // 兼容旧版诊断编号。倒地由 robot.cc 统一判断。
    kYawRunaway,    // |偏航角速度| > 18 rad/s 持续 > 25 ms：偏航符号/轮子接反的典型症状
    kWheelStale,    // 轮子状态 > 150 ms 没刷新
    kWheelRunaway,  // 轮速 > 0.8 m/s 持续 > 100 ms（拿起/空转/已经失控）
    kSaturated,     // 两轮指令连续 0.5 s 在 95% 以上
    kNumeric,       // NaN / inf
    kLifted,        // 轮子转得快、机身几乎没动：被拿起来了（空转）
};

inline const char* GuardName(Guard g) {
    switch (g) {
        case Guard::kNone: return "none";
        case Guard::kImu: return "imu";
        case Guard::kGap: return "gap";
        case Guard::kTilt: return "tilt";
        case Guard::kYawRunaway: return "yaw_runaway";
        case Guard::kWheelStale: return "wheel_stale";
        case Guard::kWheelRunaway: return "wheel_runaway";
        case Guard::kSaturated: return "saturated";
        case Guard::kNumeric: return "numeric";
        case Guard::kLifted: return "lifted";
    }
    return "?";
}

// 固件侧的原始信号（robot.cc 每个控制周期整理一次）
struct Signals {
    int64_t now_us = 0;
    bool imu_ok = false;           // isIMUInit
    float pitch_deg = 0;           // imu.cc: pitch
    float dq_dps = 0;              // imu.cc: dq（俯仰角速度，度/秒）
    float yaw_rate_dps = 0;        // imu.cc: imu_yaw_rate_dps()，已减零偏，yaw 增大（左转）为正
    float yaw_deg = 0;             // imu.cc: yaw（积分偏航，度）
    float q_head_deg = 0;          // 头部舵机角度（度），固件里的 q_head
    float imu_zero_deg = 0;        // robot.cc: imu_zero
    float wheel1_counts = 0;       // motor.cc: wheel1_x（累计编码器计数，前进为正，1024 计数/圈）
    float wheel2_counts = 0;       // motor.cc: wheel2_x（同上，轮 2 已取反）
    int64_t wheel1_us = 0;         // 轮 1 状态最近一次刷新的时刻（0 = 还没收到过）
    int64_t wheel2_us = 0;
    float wheel_x_m = 0;           // robot.cc: wheel_x（里程，米，D = 60 mm）
    float stable_pos_m = 0;        // robot.cc: stable_pos（位置目标，米）
    float stable_yaw_deg = 0;      // robot.cc: stable_yaw（偏航目标，度，与 yaw - q_head 比较）
    float joy_vx = 0;              // 摇杆（±100，0 表示没用摇杆）：非零时覆盖位置/角度外环
    float joy_vyaw = 0;            // 正 = 左转
    bool head_moving = false;      // 头部舵机正在转动（斜坡/动作中）：此时策略改用轮子差分偏航角速度，避免头部自转污染观测
};

struct Output {
    bool run = false;              // false：本周期没有策略输出（起步第一拍、间隔过大、轮子还没刷新……）
    bool stop = false;             // true：请求机器人停机（robot_state = 0）
    bool latched = false;          // 偏航失控连续出现，已锁定输出 0，需要 rearm()
    bool cancel_move = false;      // true：受阻，固件应把 stable_pos 设成当前位置（放弃移动），航向目标不动
    bool gave_up = false;          // 只在做出“放弃”决定的那个周期为 true（计数/日志用）
    bool stood = false;            // 只在“站稳了”的那个周期为 true（每次起步一次）：起步后连续 stand_ok_s 秒稳定且已完成力矩接管，固件据此播放“可以松手了”
    bool reanchor = false;         // true：摇杆在用（或刚松开、指令还没衰减完），固件应把 stable_pos/stable_yaw 设成当前位置/航向，松手后原地停住而不是弹回
    Guard guard = Guard::kNone;
    float ramp = 0;                // 0..1：起步 0.25 s 内策略输出的放开比例（run == false 时为 0）。固件可以按 ramp 与原控制器平滑切换
    float tor1 = 0, tor2 = 0;      // 总线单位（sendWheelTor），已乘 ramp
    float left = 0, right = 0;     // 策略的左右轮指令（前进为正，±1000），已乘 ramp
    // 诊断
    float pitch_rad = 0, pitch_rate = 0, yaw_rate = 0, v_wheel = 0, v_cmd = 0, r_cmd = 0;
};

// 运行参数（默认值就是量产值；仿真桥可以改，用来做对照实验；wheel1_is_left 可在线切换）
struct Config {
    bool wheel1_is_left = true;        // 轮 1（tor1）对策略来说是左轮。由偏航回路符号推出，若实机转向反了就设成 false
    float ramp_s = 0.25f;              // 起步后输出线性放开的时间
    float wheel_alpha = 0.35f;         // 轮速 EMA 的新样本权重。训练时用的是“瞬时速度 + 0.7/0.3 EMA”；位置差分本身多 T/2 的延迟，
                                       // 所以权重取 0.35 让总延迟与训练一致（仿真对比见 docs/INTEGRATION.md）
    float yaw_guard_rad_s = 18.0f;     // 偏航角速度保护阈值（策略自己在静摩擦“打滑”时会冲到 10+ rad/s，所以不能设得太紧）
    float nav_deadband_m = 0.005f;     // 位置外环死区：轮子有静摩擦，没有死区会在目标附近来回“打摆”（站立俯仰抖动 1.4° -> 1.1°）
    float nav_deadband_yaw_deg = 0.0f; // 航向外环死区
    float nav_kp_x = 1.5f;             // 位置外环增益 [1/s]（与 Navigator::kKpX 相同）
    float nav_v_max = 0.07f;           // 位置外环给出的最大速度 [m/s]（0.10 时撞上障碍的冲击力大约高 40%；你说过 0.05 m/s 就是正常速度）
    float nav_kp_yaw = 1.0f;           // 航向外环增益 [1/s]（第一版是 2.0）
    float nav_r_max = 0.8f;            // 航向外环给出的最大偏航角速度 [rad/s]（第一版是 1.5）
    float nav_r_slew = 1.0f;           // 外环偏航角速度指令的变化率限制 [rad/s^2]；0 = 不加（策略内部本来就有 4 rad/s^2 的斜坡）
    // 死区补偿：电机在小力矩下转不动（死区 / 静摩擦），策略的积分项会一点点攒力矩、突然把轮子顶开、冲过头、再反向——站着不动时就是周期约 1.4 s 的来回晃
    // （真机 144 s：俯仰 ±0.5°，轮速 ±1.3 cm/s，位置 ±5 mm）。在输出力矩的方向上加 dz_comp * tanh(u / dz_tau)（奇函数，零附近连续）。
    // 仿真（48 台随机机器人）：站着的抖动 俯仰 0.83° → 0.54°、轮速 1.9 → 1.2 cm/s、位置来回 29 → 19 mm；被推之后的恢复不变差（0/48 摔倒，位置游走 131 → 119 mm）；
    // 补偿量 20 起力矩抖动明显变大、最差十分之一的机器人（摩擦小的）变差。0 = 关。起步斜坡期间按斜坡比例加。
    float dz_comp = 6.0f;              // [指令单位]；调试页变量 27。仿真里 12 最好，真机上 6 和 12 的位置/俯仰抖动一样（降到不补偿的 1/2–1/3），6 的力矩抖动最小（20 对 31）
    float dz_tau = 6.0f;               // [指令单位]
    // 补偿只跟随力矩需求的“慢”部分：真机高速记录里（/api/trace）补偿开着时有时会出现一个 15–25 Hz 的极限环（力矩 ±17 个单位，补偿 12 时 ±35，补偿 6 约 10–20% 的时间），
    // 是补偿在零附近的陡峭增益 + 电机/传感器延迟形成的；补偿关掉时没有，补偿 + 力矩死区 10 也没有但慢晃更大。把补偿的输入先低通 dz_lpf_ms：
    // 17 Hz 处的环路增益衰减到约 0.3，慢的静摩擦挣脱（约 1.2 Hz）不受影响。0 = 不滤波（第六、七版的行为）。
    float dz_lpf_ms = 30.0f;           // [ms]；调试页变量 31
    // 补偿只加在共模（平衡）分量上，不碰差模（转向）分量：补偿每个轮子会让静摩擦挣脱的“急转”更猛（仿真转向峰值角速度 3.2 → 3.7 rad/s，转圈 2.9 → 3.6）。
    // 试过但在真机上没有用、已经删掉的办法（详见 docs/INTEGRATION.md 10.8 / 10.9）：力矩死区（压得住 17 Hz 颤抖，但慢晃更大）、力矩低通、俯仰角速度低通、差模变化率限制、站稳后力矩归零的“保持”（零力矩下俯仰以 1–2°/s 漂移，留不住）。
    float diff_cap = 80.0f;            // 差模分量的幅度上限 [指令单位]；0 = 不限。压低静摩擦挣脱后的“急转”：真机 −90° 方向峰值 4.7–5.1 -> 3.3–3.7 rad/s（70：2.5–2.8，60 开始冲过目标）；
                                       // 摩擦大的机器人可能转不到位（仿真里 80 时有平均 10° 的转角误差），不合适就在调试页变量 25 调大或设 0
    bool nav_enabled = true;           // 外环（位置保持、move/rotate）；关掉就只剩摇杆指令，空闲时 v_cmd = r_cmd = 0
    // 保护阈值（默认就是量产值；仿真桥可以改，用来做对照实验）
    // 轮速保护分两档：很快的轮速持续一小会儿（拿起空转 / 策略失控）、或者中等轮速持续很久（拿起后一直转）就停机。
    // 被推了一下之后机器人要追上去救回来，轮速会短暂到 1 m/s 以上（仿真里手指级的推力就会，持续不到 0.7 s）：以前的单档 0.8 m/s 持续 0.1 s 把这些本来救得回来的情况也卸力了。
    float wheel_runaway_ms = 1.5f;     // [m/s] 轮速超过它持续 wheel_runaway_s 就停机
    float wheel_runaway_s = 0.10f;     // [s]
    float wheel_over_ms = 0.8f;        // [m/s] 轮速超过它持续 wheel_over_s 就停机
    float wheel_over_s = 1.0f;         // [s]
    float sat_cmd = 950.0f;            // 两轮指令的幅度都超过它持续 sat_s 就停机（饱和）
    float sat_s = 0.5f;                // [s]
    // 拿起检测（RL 模式）：轮子转得快（|轮速| > lift_v）而机身几乎没动（|俯仰| < lift_pitch_deg 且 |俯仰角速度| < lift_dq_dps）持续 lift_s 就停机。
    // 站在地上的机器人轮子要加速，机身一定跟着倾斜 / 转动，否则就倒了；拿在手里空转时机身还是直直的。
    // 真机记录（2026-11-16，满电）：第十七版 12 次拿起，这个状态每次都持续 ≥ 253 ms；第十八版（6° / 50°/s）36 次关电机里 33 次是它关的，轮子起转后中位 0.14 s（最长 0.29 s），
    // 剩下 3 次是手把机器人斜着拿（俯仰 12–17°、侧倾 18–44°，被老的速度保护在 0.4–1.5 s 后关掉）。拿在手里的自然倾斜就有 6–7°，6° 太贴近了：放宽到 10° / 60°/s，
    // 用精确回放（真实的控制周期时间）核对：18 段推扰、10 段干净的推 / 恢复记录里这个条件最长只成立 19 ms（判据要 100 ms），一次都没有误判；那 3 次斜着拿能抓住 2 次（早停 1.3 s 和 0.4 s）；
    // 仿真里头部推扰矩阵（5 个高度 × 6 档推力 × 24 台）、标准场景、导航、起步和关掉它完全一样；再放宽（12° / 60°/s）能抓住第 3 次，但仿真里 12 cm 高、1 N 的推少站住 1 台，没要。
    // 原来靠速度停（> 1.5 m/s 持续 0.1 s）：拿起后平均又转了 0.5 s，最长 1.7 s。lift_v = 0 关掉。调试页变量 33 / 34 / 35。
    float lift_v = 0.4f;               // [m/s]
    float lift_pitch_deg = 10.0f;
    float lift_dq_dps = 60.0f;
    float lift_s = 0.10f;              // [s]
    // 大幅推扰的“接住”辅助：策略训练时最大只推过 0.6 N（≤ 0.1 s），更大的推力下它的反应偏弱（训练的倒地判据是 46°，俯仰输入 43° 就饱和、轮速输入 0.5 m/s 就饱和；
    // 仿真里推头部（轴上 10 cm）1.5 N、0.1 s 时力矩峰值只有 350–600 / 1000）。倾倒预测量 lean = 俯仰 + assist_tau_s × 俯仰角速度 超过 assist_start_deg 之后，
    // 共模力矩的幅度至少是 assist_gain × (|lean| - assist_start_deg)（朝倒下的方向，封顶 1000）。小推扰（lean 不到 start）完全不受影响。
    // 参数是在仿真里扫出来的（随机机器人 12 台 × 推力 0.5–4 N × 推的高度 0–12 cm 共 420 种）：不加 183 个救回来，(35°, 20, 0.1 s) 218 个、小推力一个都没少；
    // start 30–40° / gain 20 / tau 0.1 是一片平台（218 / 218 / 209），gain 40 配 tau 0.1 在 start 25–35° 时过冲、小推力也会倒，所以取平台中间。start = 0 或 gain = 0 就是关。调试页变量 30 / 31。
    float assist_start_deg = 35.0f;
    float assist_gain = 20.0f;         // [指令单位 / 度]
    float assist_tau_s = 0.1f;
    // 站稳提示：起步时同步检查机器人是否连续 stand_ok_s 秒保持稳定（|俯仰| < stand_ok_pitch_deg、|俯仰角速度| < stand_ok_dq_dps、|轮速| < stand_ok_v、|偏航角速度| < stand_ok_yaw_dps），
    // Output::stood 在那一个周期为 true；每次起步只触发一次（reset() 之后才会再触发）。手扶着站稳也算——这正是要告诉用户“可以松手了”的时刻。
    // 阈值按真机的“慢晃”状态定（机器人在晃，但是站得住）：长测里慢晃时 0.6 s 窗口内的最大值中位数 / p99 是 |俯仰| 1.35 / 2.9°、|俯仰角速度| 8.3 / 22°/s、|轮速| 0.023 / 0.052 m/s、|偏航角速度| 9.6 / 16.6°/s，
    // 这组阈值让 97–98% 的慢晃窗口算“稳”（安静状态 100%），所以不会等到完全安静（刚站起来的头半分钟有时在慢晃）才响；手在晃 / 机器人在倒时这些量远大于阈值。
    bool stand_ok_enable = true;
    float stand_ok_s = 0.25f;          // 默认与力矩接管并行；动作仍不稳时从下一次稳定起重新计时。
    float stand_ok_pitch_deg = 8.0f;   // 不用太小：有的机器人（质心偏）站着的时候俯仰本来就离 imu_zero 有几度，策略自己会补偿
    float stand_ok_dq_dps = 20.0f;
    float stand_ok_v = 0.08f;          // [m/s]
    float stand_ok_yaw_dps = 40.0f;
    // 碰到障碍就停：外环要求走（|v_cmd| >= stall_v_cmd），可是最近 stall_window_s 秒（滑动窗口，每 50 ms 判断一次）低通后的轮子位置朝目标方向
    // 挪动不到 stall_progress_m（正常走路 2 s 走 14 cm）：放弃这次移动——位置目标拉回当前位置（固件据 cancel_move 处理），清掉策略里的速度/位置误差积分，
    // 然后就站在那里，不后退、不再往前顶。之后只要还有人在要求朝同一个方向走（手机遥控按住时每 0.8 s 来一步）就一直挡着，位置目标一直跟着机器人；
    // 停止命令（clearBlock）、朝另一个方向走、或者 stall_cooldown_s 内没有新的同方向请求，才解除。
    // 只看位移：轮子力矩的条件（第二版）在真机上达不到，前倾的条件（第三/四版）在仿真里没有它也一样干净（空桌面 1/192 对 0/192，撞墙最后 2 s 的力 3.1 对 8.7 N），去掉了。
    // 位置先经过 stall_lpf_s 的低通：顶着障碍来回弹跳时轮子位置在摆动，不能把摆动的某一个瞬间当成“有进展”。
    // 进展阈值 4 cm：真机压着软障碍时还会蹭着往前挪 3 cm、弹跳来回 4 cm，1.5 cm 的固定窗口让放弃推迟到碰上之后 8 s。
    // 请求开始后机器人一直没走出 stall_progress_m 的话，要持续 stall_start_s 才算（起步慢的机器人头 2 s 也没进展，会被误判）；起步后 stall_arm_s 内不判断（站起来的头几秒机器人自己还在晃）。
    // 停止命令之后 stall_settle_s 内位置目标也跟着机器人走，直到它稳住（轮速 < 2 cm/s 且俯仰偏差 < 3°）或者来了新的移动请求（目标和位置差 >= stall_new_request_m）：
    // 不然保持位置会把被障碍弹开的机器人又拉回去撞（真机上松手时机器人正在弹，之后摆动了 4 s 以上）。
    // 试过“弹回来就立刻放弃”（轮速刚朝目标方向走，随后被反向弹回），空桌面上 33% 误判；试过放弃后朝反方向退一小段，真机上轮子没动，用户也不要退，去掉了。
    bool stall_enable = true;
    float stall_v_cmd = 0.05f;         // [m/s]
    float stall_window_s = 2.0f;       // 滑动窗口：最近这么多秒
    float stall_progress_m = 0.04f;    // [m]
    float stall_start_s = 3.5f;        // [s]
    float stall_lpf_s = 0.4f;          // [s]
    float stall_cooldown_s = 4.0f;     // [s]；手机经云端发来的步进会有抖动，两步之间超过 2 s 并不少见（我的读数卡一下就有 2.17 s）
    float stall_arm_s = 3.0f;          // [s]
    float stall_settle_s = 2.0f;       // [s]
    float stall_new_request_m = 0.03f; // [m]
};

class RlController {
public:
    Config cfg;
    static constexpr float kTyreRadiusM = 0.0308f;            // STEP：轮胎外径 61.6 mm
    static constexpr float kWheelTrackM = 0.0366f;            // 左右轮胎接地点中心距（hover.xml：2 × 18.3 mm）
    static constexpr float kMetersPerCount = kTyreRadiusM * 6.2831853f / 1024.0f;
    static constexpr float kDegToRad = 0.017453293f;
    static constexpr float kWheelStaleS = 0.15f;
    static constexpr float kWheelCountJump = 500.0f;          // 一个轮子状态帧之间超过这么多个计数当作错帧丢掉（真机约 39 ms 一帧，500 个计数 = 2.4 m/s；原来的 300 在 1.45 m/s 以上就冻结了速度估计）
    static constexpr float kGapS = 0.04f;
    static constexpr float kHeadMotionHoldS = 0.35f;          // 头部停止转动后继续用轮速偏航角速度的尾窗（覆盖舵机减速与读数延迟）
    static constexpr float kHeadMoveDeltaDeg = 0.8f;          // q_head 读数跳变阈值：覆盖未设 head_moving 的手势/外部转头

    // 机器人不在平衡状态（或不使用 RL）时每个周期调用，让下一次起步是干净的
    void reset() {
        policy_.reset();
        have_last_ = false;
        w1_seen_ = w2_seen_ = false;
        v1_f_ = v2_f_ = 0.0f;
        started_us_ = 0;
        t_yaw_over_ = t_wheel_over_ = t_wheel_slow_ = t_sat_ = t_lift_ = 0.0f;
        follow_ = false;
        r_nav_ = 0.0f;
        block_dir_ = 0.0f;
        block_until_us_ = 0;
        x_lpf_init_ = false;
        stall_dir_ = 0.0f;
        ringReset();
        req_t_ = 0.0f;
        moved_ = false;
        settle_until_us_ = 0;
        settle_calm_t_ = 0.0f;
        cl_ = 0.0f;
        stood_ = false;
        stand_since_us_ = 0;
        q_head_seen_ = false;
        prev_q_head_deg_ = 0.0f;
        head_motion_hold_s_ = 0.0f;
    }

    // 清除偏航失控锁定
    void rearm() {
        latched_ = false;
        runaway_count_ = 0;
        reset();
    }
    bool latched() const { return latched_; }
    void clearBlock() { clear_block_req_.store(true); }   // 停止命令时调用（别的任务）：下一次按下前进键不再被上一次的“挡住”拦着
    void clearIntegrals() { policy_.clearPositionIntegrals(); }   // 停止命令时调用：撞墙/按住前进时积出来的推力不要再带到松手之后

    Output step(const Signals& s) {
        Output o;
        if (clear_block_req_.exchange(false)) {
            block_dir_ = 0.0f;
            block_until_us_ = 0;
            settle_until_us_ = s.now_us + (int64_t)(cfg.stall_settle_s * 1e6f);
            settle_calm_t_ = 0.0f;
        }
        if (latched_) {
            o.latched = true;
            o.guard = Guard::kYawRunaway;
            return o;
        }
        if (!numericOk(s) || !s.imu_ok) {
            o.guard = s.imu_ok ? Guard::kNumeric : Guard::kImu;
            o.stop = (o.guard == Guard::kNumeric);
            have_last_ = false;
            stand_since_us_ = 0;
            return o;
        }
        // ---- 时间
        if (!have_last_) {
            seed(s);
            return o;  // 第一拍只起步估计器
        }
        float dt = (float)(s.now_us - last_us_) * 1e-6f;
        last_us_ = s.now_us;
        if (dt > kGapS || dt <= 0.0f) {
            seed(s);
            o.guard = Guard::kGap;
            return o;
        }
        dt = clampf(dt, 0.003f, 0.012f);

        // ---- 轮速：每个轮子在自己的刷新时刻用编码器差分，再做 EMA
        updateWheel(s.wheel1_counts, s.wheel1_us, w1_cnt_, w1_us_, w1_seen_, v1_f_, cfg.wheel_alpha);
        updateWheel(s.wheel2_counts, s.wheel2_us, w2_cnt_, w2_us_, w2_seen_, v2_f_, cfg.wheel_alpha);
        if (!w1_seen_ || !w2_seen_ || (float)(s.now_us - s.wheel1_us) * 1e-6f > kWheelStaleS ||
            (float)(s.now_us - s.wheel2_us) * 1e-6f > kWheelStaleS) {
            stand_since_us_ = 0;
            o.guard = Guard::kWheelStale;
            o.stop = started_us_ != 0;  // 刚起步时轮子还没刷新过，只是不输出
            return o;
        }
        float v_wheel = 0.5f * (v1_f_ + v2_f_);
#ifdef HOVER_RL_TEST_HOOKS
        if (test_v_override == test_v_override) v_wheel = test_v_override;   // 仿真桥专用：用仿真器自己的轮速做对照
#endif

        // ---- 姿态：imu.cc 已在头部系内扣除三维安装倾角后再按 q_head 投影到机身系，
        //      因此 pitch 与 q_head 无关，imu_zero（及动作倾角偏置）直接作为机身俯仰零位。
        const float pitch_ref = s.imu_zero_deg;
        const float pitch_rel_deg = s.pitch_deg - pitch_ref;

        // ---- 偏航角速度：陀螺装在头上，机身转动时直接用原始读数；头部舵机转动期间
        //      （head_moving 或 q_head 跳变后的短尾窗内）策略改用双轮编码器差分推算的机身偏航角速度，
        //      避免 ±150~200°/s 的头转速度打爆差模力矩和 er_int 积分。偏航失控保护仍看原始陀螺。
        const float yaw_rate = s.yaw_rate_dps * kDegToRad;
        if (s.head_moving || (q_head_seen_ && fabsf(s.q_head_deg - prev_q_head_deg_) > kHeadMoveDeltaDeg)) {
            head_motion_hold_s_ = kHeadMotionHoldS;
        } else {
            head_motion_hold_s_ = fmaxf(0.0f, head_motion_hold_s_ - dt);
        }
        prev_q_head_deg_ = s.q_head_deg;
        q_head_seen_ = true;
        const float v_left = cfg.wheel1_is_left ? v1_f_ : v2_f_;
        const float v_right = cfg.wheel1_is_left ? v2_f_ : v1_f_;
        const float policy_yaw_rate = (head_motion_hold_s_ > 0.0f)
            ? ((v_right - v_left) / kWheelTrackM)
            : yaw_rate;

        // 轮子位置的低通（顶住检测用）
        if (!x_lpf_init_ || cfg.stall_lpf_s <= 0.0f) {
            x_lpf_ = s.wheel_x_m;
            x_lpf_init_ = true;
        } else {
            x_lpf_ += (s.wheel_x_m - x_lpf_) * clampf(dt / cfg.stall_lpf_s, 0.0f, 1.0f);
        }

        // ---- 命令：摇杆优先，否则由固件已有的位置/角度目标（MCP move/rotate、双击转 360°）经外环给出
        //      摇杆松开后，策略内部的速度/角速度指令还要按限幅斜率衰减到 0；这段时间继续“跟随”（reanchor），
        //      这样松手后机器人就停在停下来的地方，而不是被外环拉回摇杆开始前的位置。
        float v_cmd, r_cmd;
        const bool joy_active = fabsf(s.joy_vx) > 0.5f || fabsf(s.joy_vyaw) > 0.5f;
        if (joy_active) {
            follow_ = true;
        } else if (follow_ && fabsf(policy_.speedCommand()) < 0.003f && fabsf(policy_.yawRateCommand()) < 0.02f) {
            follow_ = false;
        }
        if (follow_) {
            v_cmd = clampf(s.joy_vx / 100.0f, -1.0f, 1.0f) * Policy::kV_MAX;
            r_cmd = clampf(s.joy_vyaw / 100.0f, -1.0f, 1.0f) * Policy::kR_MAX;
            r_nav_ = r_cmd;
            o.reanchor = true;
        } else if (cfg.nav_enabled) {
            // 位置/航向外环（与 Navigator 同一个 P 律，增益和限幅是运行时参数）
            const float base_yaw_deg = s.yaw_deg - s.q_head_deg;   // 固件的定义：机身偏航 = yaw - q_head
            const float x_err = deadband(s.stable_pos_m - s.wheel_x_m, cfg.nav_deadband_m);
            const float yaw_err_deg = deadband(s.stable_yaw_deg - base_yaw_deg, cfg.nav_deadband_yaw_deg);
            v_cmd = clampf(cfg.nav_kp_x * x_err, -cfg.nav_v_max, cfg.nav_v_max);
            const float r_raw = clampf(cfg.nav_kp_yaw * yaw_err_deg * kDegToRad, -cfg.nav_r_max, cfg.nav_r_max);
            if (cfg.nav_r_slew > 0.0f) {
                r_nav_ += clampf(r_raw - r_nav_, -cfg.nav_r_slew * dt, cfg.nav_r_slew * dt);
            } else {
                r_nav_ = r_raw;
            }
            r_cmd = r_nav_;
            const bool stall_armed = started_us_ != 0 && (float)(s.now_us - started_us_) * 1e-6f >= cfg.stall_arm_s;
            if (cfg.stall_enable && stall_armed) {
                // 停止命令之后：位置目标跟着机器人，直到它稳住，或者来了新的移动请求
                if (settle_until_us_ != 0) {
                    if (s.now_us >= settle_until_us_ || fabsf(s.stable_pos_m - s.wheel_x_m) >= cfg.stall_new_request_m) {
                        settle_until_us_ = 0;
                    } else {
                        if (fabsf(v_wheel) < 0.02f && fabsf(pitch_rel_deg) < 3.0f) settle_calm_t_ += dt; else settle_calm_t_ = 0.0f;
                        if (settle_calm_t_ >= 0.3f) {
                            settle_until_us_ = 0;
                        } else {
                            o.cancel_move = true;
                            v_cmd = 0.0f;
                        }
                    }
                }
                if (fabsf(v_cmd) >= cfg.stall_v_cmd) {
                    const float dir = v_cmd > 0.0f ? 1.0f : -1.0f;
                    if (dir != stall_dir_) {
                        stall_dir_ = dir;
                        ringReset();
                        req_t_ = 0.0f;
                        req_x0_ = x_lpf_;
                        moved_ = false;
                    }
                    if (block_dir_ != 0.0f && dir != block_dir_) block_dir_ = 0.0f;       // 朝另一个方向走：不再挡着
                    req_t_ += dt;
                    if ((x_lpf_ - req_x0_) * dir >= cfg.stall_progress_m) moved_ = true;
                    ring_dt_acc_ += dt;
                    if (ring_dt_acc_ >= kRingDt) {
                        ring_dt_acc_ -= kRingDt;
                        ringPush(x_lpf_);
                        if (ring_n_ >= ringTarget() && (moved_ || req_t_ >= cfg.stall_start_s)) {
                            const float progress = (ringX(ring_n_ - 1) - ringX(0)) * dir;
                            if (progress < cfg.stall_progress_m && !(block_dir_ == dir && s.now_us < block_until_us_)) {
                                giveUp(dir, s.now_us, o);
                            }
                        }
                    }
                    if (block_dir_ == dir && s.now_us < block_until_us_) {
                        o.cancel_move = true;
                        v_cmd = 0.0f;
                        // 手机遥控按住时每 0.8 s 又来一步：只要还有人在要求朝这个方向走，就一直挡着（松手 = stop 命令 = 解除），不然每隔几秒又撞一次
                        block_until_us_ = s.now_us + (int64_t)(cfg.stall_cooldown_s * 1e6f);
                    }
                } else {
                    stall_dir_ = 0.0f;
                    ringReset();
                    req_t_ = 0.0f;
                    moved_ = false;
                }
                if (block_dir_ != 0.0f && s.now_us >= block_until_us_) block_dir_ = 0.0f;
                // 挡住期间位置目标一直跟着机器人（不管这一周期有没有新的请求）：被障碍弹开几厘米也不会再被拉回去撞
                if (block_dir_ != 0.0f && s.now_us < block_until_us_) o.cancel_move = true;
            }
        } else {
            v_cmd = r_cmd = 0.0f;
            r_nav_ = 0.0f;
        }

        // ---- 策略
        Inputs in{pitch_rel_deg * kDegToRad, s.dq_dps * kDegToRad, policy_yaw_rate, v_wheel, v_cmd, r_cmd, dt};
        Outputs pol = policy_.step(in);
        if (cfg.diff_cap > 0.0f) {
            // 差模（右 - 左）/2 决定偏航加速度：限制它的幅度，让轮子从静摩擦里“挣脱”时不至于一下子转得太猛
            const float common = 0.5f * (pol.left + pol.right);
            const float diff = clampf(0.5f * (pol.right - pol.left), -cfg.diff_cap, cfg.diff_cap);
            pol.left = common - diff;
            pol.right = common + diff;
        }
        if (cfg.assist_start_deg > 0.0f && cfg.assist_gain > 0.0f) {
            const float lean = pitch_rel_deg + cfg.assist_tau_s * s.dq_dps;
            const float over = fabsf(lean) - cfg.assist_start_deg;
            if (over > 0.0f) {
                const float u = (lean > 0.0f ? 1.0f : -1.0f) * clampf(cfg.assist_gain * over, 0.0f, 1000.0f);
                float common = 0.5f * (pol.left + pol.right);
                const float diff = 0.5f * (pol.right - pol.left);
                if (u * common < 0.0f || fabsf(u) > fabsf(common)) common = u;
                pol.left = common - diff;
                pol.right = common + diff;
            }
        }
        if (started_us_ == 0) started_us_ = s.now_us;
        const float ramp = clampf(cfg.ramp_s > 0.0f ? (float)(s.now_us - started_us_) * 1e-6f / cfg.ramp_s : 1.0f, 0.0f, 1.0f);
        o.left = pol.left * ramp;
        o.right = pol.right * ramp;
        o.pitch_rad = in.pitch_rad;
        o.pitch_rate = in.pitch_rate_rad_s;
        o.yaw_rate = yaw_rate;
        o.v_wheel = v_wheel;
        o.v_cmd = policy_.speedCommand();
        o.r_cmd = policy_.yawRateCommand();
        if (!fin(o.left) || !fin(o.right)) {
            o.left = o.right = 0.0f;
            stand_since_us_ = 0;
            o.guard = Guard::kNumeric;
            o.stop = true;
            return o;
        }

        // ---- 保护
        Guard g = Guard::kNone;
        t_yaw_over_ = (fabsf(yaw_rate) > cfg.yaw_guard_rad_s) ? t_yaw_over_ + dt : 0.0f;
        if (t_yaw_over_ > 0.025f) g = Guard::kYawRunaway;
        t_wheel_over_ = (fabsf(v_wheel) > cfg.wheel_runaway_ms) ? t_wheel_over_ + dt : 0.0f;
        if (t_wheel_over_ > cfg.wheel_runaway_s) g = Guard::kWheelRunaway;
        t_wheel_slow_ = (fabsf(v_wheel) > cfg.wheel_over_ms) ? t_wheel_slow_ + dt : 0.0f;
        if (t_wheel_slow_ > cfg.wheel_over_s) g = Guard::kWheelRunaway;
        t_sat_ = (fabsf(o.left) > cfg.sat_cmd && fabsf(o.right) > cfg.sat_cmd) ? t_sat_ + dt : 0.0f;
        if (t_sat_ > cfg.sat_s) g = Guard::kSaturated;
        t_lift_ = (cfg.lift_v > 0.0f && fabsf(v_wheel) > cfg.lift_v && fabsf(pitch_rel_deg) < cfg.lift_pitch_deg && fabsf(s.dq_dps) < cfg.lift_dq_dps) ? t_lift_ + dt : 0.0f;
        if (t_lift_ > cfg.lift_s) g = Guard::kLifted;
        if (g != Guard::kNone) {
            stand_since_us_ = 0;
            o.guard = g;
            o.stop = true;
            o.left = o.right = 0.0f;
            if (g == Guard::kYawRunaway) {
                // 偏航失控：连续 3 次就锁定，避免“起立—转飞—停机”无限循环
                if (s.now_us - last_runaway_us_ > 20000000LL) runaway_count_ = 0;
                last_runaway_us_ = s.now_us;
                if (++runaway_count_ >= 3) latched_ = true;
            }
            o.latched = latched_;
            return o;
        }

        // Count stable observations during the torque handover. The cue is due only
        // once the controller has both a continuous stable window and full authority.
        const bool steady = cfg.stand_ok_enable && fabsf(pitch_rel_deg) < cfg.stand_ok_pitch_deg &&
            fabsf(s.dq_dps) < cfg.stand_ok_dq_dps && fabsf(v_wheel) < cfg.stand_ok_v &&
            fabsf(s.yaw_rate_dps) < cfg.stand_ok_yaw_dps;
        if (!steady) {
            stand_since_us_ = 0;
        } else if (!stood_) {
            if (!stand_since_us_) stand_since_us_ = s.now_us;
            if (ramp >= 1.0f && s.now_us - stand_since_us_ >= cfg.stand_ok_s * 1e6f) {
                stood_ = true;
                o.stood = true;
            }
        }

        // ---- 映射到总线：轮 1 正为前进，轮 2 负为前进
        float c = 0.5f * (o.left + o.right);       // 共模（前进为正）
        const float d = 0.5f * (o.right - o.left); // 差模
        // 死区补偿（只加在共模上；输入先低通，见 Config::dz_comp / dz_lpf_ms）
        if (cfg.dz_lpf_ms > 0.0f) {
            cl_ += (c - cl_) * dt / (cfg.dz_lpf_ms * 1e-3f + dt);
        } else {
            cl_ = c;
        }
        if (cfg.dz_comp > 0.0f && cfg.dz_tau > 0.0f) {
            c += cfg.dz_comp * ramp * tanhf(cl_ / cfg.dz_tau);
        }
        const float l = clampf(c - d, -1000.0f, 1000.0f);
        const float r = clampf(c + d, -1000.0f, 1000.0f);
        o.tor1 = cfg.wheel1_is_left ? l : r;
        o.tor2 = -(cfg.wheel1_is_left ? r : l);
        o.ramp = ramp;
        o.run = true;
        return o;
    }

    const Policy& policy() const { return policy_; }
#ifdef HOVER_RL_TEST_HOOKS
    float test_v_override = __builtin_nanf("");
#endif

private:
    static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    static float deadband(float e, float d) { return e > d ? e - d : (e < -d ? e + d : 0.0f); }

    static bool fin(float x) { return __builtin_isfinite(x); }

    static bool numericOk(const Signals& s) {
        return fin(s.pitch_deg) && fin(s.dq_dps) && fin(s.yaw_rate_dps) && fin(s.yaw_deg) && fin(s.q_head_deg) &&
               fin(s.imu_zero_deg) && fin(s.wheel1_counts) && fin(s.wheel2_counts) && fin(s.wheel_x_m) &&
               fin(s.stable_pos_m) && fin(s.stable_yaw_deg) && fin(s.joy_vx) && fin(s.joy_vyaw);
    }

    void seed(const Signals& s) {
        stand_since_us_ = 0;
        have_last_ = true;
        last_us_ = s.now_us;
        w1_cnt_ = s.wheel1_counts;
        w2_cnt_ = s.wheel2_counts;
        w1_us_ = s.wheel1_us;
        w2_us_ = s.wheel2_us;
        w1_seen_ = (s.wheel1_us != 0);
        w2_seen_ = (s.wheel2_us != 0);
        prev_q_head_deg_ = s.q_head_deg;
        q_head_seen_ = true;
        head_motion_hold_s_ = 0.0f;
    }

    static void updateWheel(float counts, int64_t us, float& prev_cnt, int64_t& prev_us, bool& seen, float& vf, float alpha) {
        if (us == prev_us) return;
        if (seen && us != 0) {
            const float dtw = (float)(us - prev_us) * 1e-6f;
            const float dc = counts - prev_cnt;
            if (dtw > 0.010f && dtw < 0.25f && fabsf(dc) < kWheelCountJump) {
                vf = (1.0f - alpha) * vf + alpha * (kMetersPerCount * dc / dtw);
            }
        }
        prev_cnt = counts;
        prev_us = us;
        seen = (us != 0);
    }

    // 顶住检测的滑动窗口：每 50 ms 记一次（低通后的轮子位置），窗口装满 stall_window_s 秒之后每次都判断
    static constexpr int kStallRing = 64;
    static constexpr float kRingDt = 0.05f;
    int ringTarget() const {
        int n = (int)(cfg.stall_window_s / kRingDt + 0.5f) + 1;
        return n < 2 ? 2 : (n > kStallRing ? kStallRing : n);
    }
    void ringReset() {
        ring_n_ = 0;
        ring_head_ = 0;
        ring_dt_acc_ = 0.0f;
    }
    void ringPush(float x) {
        const int target = ringTarget();
        if (ring_n_ >= target) ring_n_ = target - 1;     // 最旧的一个出窗口
        ring_x_[ring_head_] = x;
        ring_head_ = (ring_head_ + 1) % kStallRing;
        ring_n_++;
    }
    float ringX(int k) const {                 // k = 0 最旧 … ring_n_-1 最新
        return ring_x_[(ring_head_ - ring_n_ + k + kStallRing) % kStallRing];
    }

    // 放弃这次移动：挡住的方向、冷却计时、清掉策略里的积分
    void giveUp(float dir, int64_t now_us, Output& o) {
        block_dir_ = dir;
        block_until_us_ = now_us + (int64_t)(cfg.stall_cooldown_s * 1e6f);
        policy_.clearPositionIntegrals();
        ringReset();
        o.gave_up = true;
    }

    Policy policy_;
    float r_nav_ = 0.0f;           // 外环偏航角速度指令（可选的斜坡限制后）
    float stall_dir_ = 0.0f;       // 顶住检测正在看的方向
    float req_t_ = 0.0f, req_x0_ = 0.0f;   // 这个方向的请求已经持续多久 / 开始时的（低通）位置
    bool moved_ = false;           // 请求开始以来机器人已经朝目标走出过 stall_progress_m
    float ring_x_[kStallRing] = {0};
    int ring_head_ = 0, ring_n_ = 0;
    float ring_dt_acc_ = 0.0f;
    float cl_ = 0.0f;              // 补偿的输入（共模力矩需求的慢部分）
    bool stood_ = false;           // 这次起步已经报过“站稳了”
    int64_t stand_since_us_ = 0;   // 首个连续稳定样本的时间。
    int64_t settle_until_us_ = 0;  // 停止命令之后位置目标跟着机器人走，直到这个时刻（0 = 不在这个状态）
    float settle_calm_t_ = 0.0f;   // 已经稳住了多久
    float block_dir_ = 0.0f;       // 碰壁后被挡住的方向（+1 前 / −1 后 / 0 没有）
    int64_t block_until_us_ = 0;
    float x_lpf_ = 0.0f;           // 轮子位置的低通
    std::atomic<bool> clear_block_req_{false};
    bool x_lpf_init_ = false;
    bool have_last_ = false;
    int64_t last_us_ = 0, started_us_ = 0;
    float w1_cnt_ = 0, w2_cnt_ = 0;
    int64_t w1_us_ = 0, w2_us_ = 0;
    bool w1_seen_ = false, w2_seen_ = false;
    float v1_f_ = 0, v2_f_ = 0;
    float t_yaw_over_ = 0, t_wheel_over_ = 0, t_wheel_slow_ = 0, t_sat_ = 0, t_lift_ = 0;
    int runaway_count_ = 0;
    int64_t last_runaway_us_ = 0;
    bool latched_ = false;
    bool follow_ = false;
    bool q_head_seen_ = false;
    float prev_q_head_deg_ = 0.0f;
    float head_motion_hold_s_ = 0.0f;
};

}  // namespace hover_rl
