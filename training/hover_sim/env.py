"""MuJoCo environment of the Hover two-wheel robot: balance + forward/backward + left/right turn.

Everything the policy sees is built the way the firmware can build it (hover_board / imu.cc / motor.cc):
  * pitch: the firmware's complementary filter (accelerometer atan weight 1/(|gyro|+200) per sample) run on simulated
    QMI8658C samples (accelerometer measured at the IMU position on the head => includes the tangential/centripetal terms)
  * pitch rate: gyro, EMA 0.6/0.4 (imu.cc)
  * wheel speed: bus reads, one wheel every ~6 control cycles, hold + EMA 0.7/0.3 (motor.cc)
  * actuation: command -> dead time -> first-order lag -> dead zone -> torque (plant.py)

Frames: x forward (screen side), y left, z up.  Pitch > 0 = leaning forward, yaw rate > 0 = turning left.
Observation (9 in the default configuration, scaled by cfg.obs_scale) and action (2, +-1 -> +-1000 command units):
  obs = [pitch_err, pitch_rate, yaw_rate, wheel_speed, speed_cmd, yaw_rate_cmd, speed_err_integral, yaw_err_integral, position_err_integral]
  action = [common, differential]:  left = common - diff, right = common + diff  (diff > 0 turns left)
"""
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

from . import plant as PL

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / 'models/hover_dyn.xml'
G = 9.81
R_TYRE = PL.TYRE_R
TRACK = 0.0366
OBS_DIM = 8
ACT_DIM = 2
# fixed input scaling (kept constant so the exported network needs no running statistics)
OBS_SCALE_FAST = np.array([1 / 0.30, 1 / 3.0, 1 / 3.0, 1 / 0.5, 1 / 0.5, 1 / 3.0, 1 / 0.3, 1 / 0.6], dtype=np.float32)
OBS_SCALE_TOY8 = np.array([1 / 0.15, 1 / 1.5, 1 / 2.0, 1 / 0.1, 1 / 0.1, 1 / 2.0, 1 / 0.1, 1 / 0.5], dtype=np.float32)   # toy2 (no position integral)
OBS_SCALE_TOY = np.concatenate([OBS_SCALE_TOY8, np.array([1 / 0.15], dtype=np.float32)])                                  # + position-error integral
OBS_SCALE = OBS_SCALE_TOY
CMD_UNITS = PL.CMD_MAX


class EnvConfig:
    """Task contract + reward weights. Default = desktop-toy contract (speed <= 0.15 m/s); ``EnvConfig.fast()`` = the first
    contract (0.5 m/s) that policy hover_balance_v6 was trained with (kept so that it can still be evaluated)."""
    contract = 'toy'
    n_obs = 9
    obs_scale = OBS_SCALE_TOY
    episode_s = 12.0
    physics_dt = 0.002
    v_max = 0.15           # m/s command range at cmd_scale 1
    r_max = 2.0            # rad/s
    a_lat_max = 1.0        # m/s^2: |v * r| cap for the commanded turn rate (static tipping limit ~4 m/s^2 with a 36.6 mm track)
    v_acc_range = (0.1, 0.4)    # m/s^2 command slew, drawn per episode (the firmware wrapper uses one fixed value inside)
    r_acc_range = (2.0, 6.0)    # rad/s^2
    cmd_hold = (1.0, 4.0)       # s a random command is held
    speed_lp_s = 0.0       # optional low-pass on the wheel-speed input; tau = 0.1 s was tried (v8) and was clearly worse, so off
    ev_leak_s = 120.0
    er_leak_s = 30.0
    ev_clip = 0.25
    er_clip = 1.0
    ep_leak_s = 60.0       # leaky integral of the speed-error integral (= integral of the position error): overcomes wheel friction
    ep_clip = 0.4
    fall_pitch = 0.80      # rad
    fall_roll = 0.80
    push_interval = (1.0, 3.0)
    push_force = (0.05, 0.6)
    push_dur = (0.04, 0.10)
    push_h = (0.0, 0.10)        # [m] height of the pushed point above the axle (the head box reaches 0.116 m)
    push_log = False            # True: the force magnitude is log-uniform in push_force (many gentle pushes, a few hard ones)
    push_lat_max = 1e9          # [N] cap for the sideways pushes (the wheels cannot stop a roll-over, hard sideways pushes are just noise for the training)
    # reward weights (toy scale: centimetres, a few cm/s)
    w_alive = 0.5
    w_v = 0.8              # narrow Gaussian: precision near zero error
    sig_v = 0.03
    w_v2 = 0.5             # linear penalty (saturating at v_sat): gradient and cost for large errors
    v_sat = 0.15
    w_r = 0.8
    sig_r = 0.3
    w_r2 = 0.6
    r_sat = 2.0
    w_turn_v = 0.3         # extra |v| penalty while turning in place (speed command ~0, yaw command != 0)
    turn_v_sat = 0.15
    w_turn_q = 0.1         # extra pitch-rate penalty in the same situation
    mode_p = (0.2, 0.2, 0.3, 0.25, 0.05)   # command mix: stand, speed only, turn only, speed + turn, step
    w_x = 0.6              # position hold: leaky integral of (true speed - speed command), linear penalty saturating at x_sat
    x_sat = 0.15           # (a smaller saturation leaves a 5-10 cm hunting cycle without any gradient)
    w_xb = 0.3             # + narrow bonus for being within sig_x of the commanded path
    sig_x = 0.02
    x_leak_s = 40.0
    w_dact = 0.04          # (delta mean action / 0.01)^2   -- applied to the policy MEAN by the trainer
    w_act = 0.02           # (mean action / 0.3)^2
    w_q = 0.2              # (pitch rate / 0.5)^2, saturating at 4
    w_roll = 0.05
    w_pitch = 0.03         # (pitch error vs equilibrium estimate / 0.15)^2 keeps the lean modest
    fall_penalty = 20.0

    @classmethod
    def fast(cls):
        c = cls()
        c.contract = 'fast'
        c.n_obs = 8
        c.obs_scale = OBS_SCALE_FAST
        c.v_max, c.r_max, c.a_lat_max = 0.5, 3.0, 1.5
        c.v_acc_range, c.r_acc_range, c.cmd_hold = (0.5, 1.2), (6.0, 16.0), (0.8, 3.0)
        c.ev_leak_s, c.er_leak_s, c.ev_clip, c.er_clip = 60.0, 30.0, 0.8, 1.0
        c.mode_p = (0.25, 0.25, 0.2, 0.25, 0.05)
        return c


def _toy8(cls):
    c = cls()
    c.contract = 'toy8'
    c.n_obs = 8
    c.obs_scale = OBS_SCALE_TOY8
    return c


EnvConfig.toy8 = classmethod(_toy8)     # toy contract without the position-error integral input (policy toy2)


def make_config(name=None):
    """HOVER_CONTRACT=fast|toy8|toy (default toy) selects the task contract for evaluation tools."""
    import os
    name = name or os.environ.get('HOVER_CONTRACT', 'toy')
    return EnvConfig.fast() if name == 'fast' else (EnvConfig.toy8() if name == 'toy8' else EnvConfig())


class HoverEnv:
    act_dim = ACT_DIM

    @staticmethod
    def priv_dim_for(cfg):
        return cfg.n_obs + 18 + PL.PRIV_DIM

    def __init__(self, seed=0, domain=1.0, cmd_scale=1.0, model_path=DEFAULT_MODEL, config=None, corner_prob=0.3,
                 push_prob=0.6):
        self.cfg = config or EnvConfig()
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.cfg.physics_dt
        self.dt = self.cfg.physics_dt
        self.domain = float(domain)
        self.cmd_scale = float(cmd_scale)
        self.obs_scale = self.cfg.obs_scale
        self.obs_dim = self.cfg.n_obs
        self.priv_dim = self.priv_dim_for(self.cfg)
        self.corner_prob = corner_prob
        self.push_prob = push_prob
        m = self.model
        n = mujoco.mj_name2id
        O = mujoco.mjtObj
        self.b_base, self.b_head = n(m, O.mjOBJ_BODY, 'base'), n(m, O.mjOBJ_BODY, 'head')
        self.b_wl, self.b_wr = n(m, O.mjOBJ_BODY, 'wheel_l'), n(m, O.mjOBJ_BODY, 'wheel_r')
        self.g_floor = n(m, O.mjOBJ_GEOM, 'floor')
        self.g_tl, self.g_tr = n(m, O.mjOBJ_GEOM, 'tyre_l'), n(m, O.mjOBJ_GEOM, 'tyre_r')
        jh, jl, jr = n(m, O.mjOBJ_JOINT, 'head_yaw'), n(m, O.mjOBJ_JOINT, 'wheel_l'), n(m, O.mjOBJ_JOINT, 'wheel_r')
        self.qa_head, self.qa_wl, self.qa_wr = m.jnt_qposadr[jh], m.jnt_qposadr[jl], m.jnt_qposadr[jr]
        self.da_head, self.da_wl, self.da_wr = m.jnt_dofadr[jh], m.jnt_dofadr[jl], m.jnt_dofadr[jr]
        # nominal arrays for per-episode randomisation
        self._nom = {k: getattr(m, k).copy() for k in
                     ('body_mass', 'body_inertia', 'body_ipos', 'dof_damping', 'dof_frictionloss', 'geom_friction', 'geom_size')}
        self._nom_mass_total = float(m.body_mass.sum())
        self.plant = PL.Plant()
        self.t = 0.0
        self.script = None
        self._last_action = np.zeros(2, np.float32)
        self.reset()

    # ------------------------------------------------------------------ randomisation of the rigid bodies
    def _apply_body(self, p: PL.Plant):
        m = self.model
        for k, v in self._nom.items():
            getattr(m, k)[:] = v
        bm, bi, bp = m.body_mass, m.body_inertia, m.body_ipos
        bb, bh, bwl, bwr = self.b_base, self.b_head, self.b_wl, self.b_wr
        # head share
        dm = bm[bh] * (p.head_mass_scale - 1.0)
        bm[bh] += dm
        bi[bh] *= (1.0 + dm / (bm[bh] - dm)) * p.inertia_scale_head
        bm[bb] -= dm
        # rotor / stator split of the (fixed) wheel assembly
        assy = PL_WHEEL_ASSY
        rotor_new = assy * (1.0 - p.stator_share)
        for b in (bwl, bwr):
            drotor = rotor_new - bm[b]
            bi[b] *= rotor_new / bm[b]
            bm[b] = rotor_new
            bm[bb] -= drotor
        bi[bb] *= (bm[bb] / self._nom['body_mass'][bb]) * p.inertia_scale_base
        bp[bh] += np.array(p.head_com)
        bp[bb] += np.array(p.base_com)
        # total mass scale
        s = p.total_mass_scale
        bm[:] *= s
        bi[:] *= s
        # tyre / floor friction
        m.geom_friction[self.g_tl, 0] = p.mu
        m.geom_friction[self.g_tr, 0] = p.mu
        m.geom_friction[self.g_floor, 0] = p.mu
        m.geom_size[self.g_tl, 1] = p.tyre_halfwidth
        m.geom_size[self.g_tr, 1] = p.tyre_halfwidth
        m.dof_damping[self.da_wl] = p.damping
        m.dof_damping[self.da_wr] = p.damping
        # gravity tilt == floor slope seen from the floor frame
        a = np.deg2rad(p.slope_deg)
        m.opt.gravity[:] = [-G * np.sin(a), 0.0, -G * np.cos(a)]
        mujoco.mj_setConst(m, self.data)

    # ------------------------------------------------------------------ reset
    def reset(self, plant: PL.Plant = None, script=None, seed=None, pitch0=None, domain=None, yaw0=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        rng = self.rng
        if domain is not None:
            self.domain = float(domain)
        corner = rng.random() < self.corner_prob * (self.domain > 0)
        self.plant = plant if plant is not None else PL.sample(rng, self.domain, corner=corner)
        p = self.plant
        self.script = script
        self._apply_body(p)
        d, m = self.data, self.model
        mujoco.mj_resetData(m, d)
        # initial attitude: close to the equilibrium of the sampled body
        com_x = self._com_offset_x()
        theta_eq = np.arctan2(com_x, max(self._com_height(), 0.01))
        if pitch0 is None:
            span = np.deg2rad(5.0 * (0.3 + 0.7 * self.domain))
            pitch0 = theta_eq + rng.uniform(-span, span)
            if rng.random() < 0.1 * self.domain:
                pitch0 += rng.uniform(-0.25, 0.25)
        yaw0_random = rng.uniform(-np.pi, np.pi)      # always drawn, so the random stream does not depend on whether yaw0 is given
        yaw0 = yaw0_random if yaw0 is None else float(yaw0)
        cy, sy, cp, sp = np.cos(yaw0 / 2), np.sin(yaw0 / 2), np.cos(pitch0 / 2), np.sin(pitch0 / 2)
        d.qpos[0:3] = [0.0, 0.0, R_TYRE + 1e-4]
        d.qpos[3:7] = [cy * cp, -sy * sp, cy * sp, sy * cp]     # q = qz(yaw) * qy(pitch)
        d.qpos[self.qa_head] = p.head_static
        d.qvel[:] = rng.normal(0.0, 0.02, m.nv) * (1.0 if self.domain > 0 else 0.0)
        mujoco.mj_forward(m, d)
        self.theta_eq = float(theta_eq)
        # ---- plant state
        self.dt = self.cfg.physics_dt
        self.t = 0.0
        self.k = 0
        self.nbuf = 16
        self.cmd_buf = np.zeros((2, self.nbuf))
        self.delay_steps = max(1, int(round(p.delay / self.dt)))
        self.uf = [0.0, 0.0]
        self.alpha_lag = self.dt / (p.lag + self.dt)
        self.kt = (p.kt, p.kt * p.kt_asym)
        # ---- IMU state (firmware filter)
        self.imu_pitch = float(np.rad2deg(pitch0)) + p.pitch_offset_deg
        self.imu_dq = 0.0
        self.imu_yaw_rate = 0.0
        self.imu_yaw = 0.0
        self.t_imu = 0.0
        self.imu_hist = deque(maxlen=16)
        self.imu_hist.append((0.0, self.imu_pitch, 0.0, 0.0))
        self.gyro_bias = p.gyro_bias_dps
        # ---- wheel feedback state
        self.w_filt = [0.0, 0.0]            # rad/s relative to the base, firmware EMA
        self.w_pos_hold = [0.0, 0.0]        # rad
        self.w_pending = [None, None]
        self.w_commit_t = [0.0, 0.0]        # time the latest wheel sample became available (firmware: wheelN_update_us)
        tick = p.ctrl_dt
        self.t_w = [tick * p.wheel_update_ticks * 0.25, tick * p.wheel_update_ticks * 0.75]
        # ---- commands
        self.v_cmd = 0.0
        self.r_cmd = 0.0
        self.v_target = 0.0
        self.r_target = 0.0
        self.t_cmd_next = 0.0
        self.v_acc = float(rng.uniform(*self.cfg.v_acc_range))
        self.r_acc = float(rng.uniform(*self.cfg.r_acc_range))
        self.cmd_mode = 0
        self.ev_int = 0.0
        self.er_int = 0.0
        self.ep_int = 0.0
        self.pos_err = 0.0
        self.v_meas = 0.0
        self.v_obs = 0.0
        self.last_cmd = np.zeros(2)
        self._last_action = np.zeros(2, np.float32)
        # ---- disturbances
        self.push = None
        self.t_push_next = rng.uniform(*self.cfg.push_interval) if (rng.random() < self.push_prob * self.domain and script is None) else 1e9
        self.d_push_script = list(script['pushes']) if script and 'pushes' in script else []
        self.head_goal = p.head_static
        self.head_target = p.head_static
        self.t_head_next = 0.0
        d.qfrc_applied[:] = 0.0
        d.ctrl[:] = 0.0
        d.ctrl[2] = p.head_static
        self.n_ticks = 0
        self.fell = False
        self._set_commands(0.0)
        self._update_truth()
        self._ctrl_jit = 0
        return self._observe(), self._priv()

    # CoM helpers (x offset and height above the axle of the sampled robot)
    def _com_offset_x(self):
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        return float(d.subtree_com[self.b_base][0] - d.xpos[self.b_base][0])

    def _com_height(self):
        d = self.data
        return float(d.subtree_com[self.b_base][2] - d.xpos[self.b_base][2])

    # ------------------------------------------------------------------ commands
    def _set_commands(self, t):
        cfg, rng = self.cfg, self.rng
        if self.script is not None:
            v, r = self.script['cmd'](t)
            self.v_target, self.r_target = float(v), float(r)
            return
        if t < self.t_cmd_next:
            return
        s = self.cmd_scale * (0.3 + 0.7 * self.domain) if self.domain < 1 else self.cmd_scale
        mode = rng.choice(5, p=cfg.mode_p)
        vmax, rmax = cfg.v_max * s, cfg.r_max * s
        v = r = 0.0
        if mode in (1, 3):
            v = rng.uniform(-vmax, vmax)
            if rng.random() < 0.3:
                v = float(np.sign(v)) * vmax
        if mode in (2, 3):
            r = rng.uniform(-rmax, rmax)
            if rng.random() < 0.3:
                r = float(np.sign(r)) * rmax
        if mode == 4:
            v = float(rng.choice([-1, 1])) * vmax
            r = 0.0
        self.v_target, self.r_target = float(v), float(r)
        self.t_cmd_next = t + rng.uniform(*cfg.cmd_hold)

    def _slew_commands(self, dt):
        cfg = self.cfg
        dv = float(np.clip(self.v_target, -cfg.v_max, cfg.v_max)) - self.v_cmd
        lim = self.v_acc * dt
        self.v_cmd += float(np.clip(dv, -lim, lim))
        # keep the lateral acceleration v*r below the tipping limit
        r_cap = min(cfg.r_max, cfg.a_lat_max / max(abs(self.v_cmd), 0.12))
        r_t = float(np.clip(self.r_target, -r_cap, r_cap))
        dr = r_t - self.r_cmd
        lim = self.r_acc * dt
        self.r_cmd += float(np.clip(dr, -lim, lim))

    # ------------------------------------------------------------------ truth
    def _update_truth(self):
        d = self.data
        Rf = np.empty(9)
        mujoco.mju_quat2Mat(Rf, d.qpos[3:7])
        R = Rf.reshape(3, 3)
        gdir = self.model.opt.gravity / np.linalg.norm(self.model.opt.gravity)
        zg = -gdir
        f = R[:, 0]
        l = R[:, 1]
        self.pitch = float(-np.arcsin(np.clip(f @ zg, -1, 1)))
        self.roll = float(np.arcsin(np.clip(l @ zg, -1, 1)))
        wb = d.qvel[3:6]                      # body frame angular velocity
        self.q = float(wb[1])
        wworld = R @ wb
        self.r_true = float(wworld @ zg)
        self.roll_rate = float(wb[0])
        fh = f - (f @ zg) * zg
        fh /= max(np.linalg.norm(fh), 1e-6)
        lh = np.cross(zg, fh)
        vw = d.qvel[0:3]
        self.v_true = float(vw @ fh)
        self.v_lat = float(vw @ lh)
        self.heading = float(np.arctan2(fh[1], fh[0]))
        self.wl_rel = float(d.qvel[self.da_wl])
        self.wr_rel = float(d.qvel[self.da_wr])
        self.head_angle = float(d.qpos[self.qa_head])

    # ------------------------------------------------------------------ sensing
    def _imu_sample(self, t):
        p, d = self.plant, self.data
        rng = self.rng
        acc = d.sensordata[0:3]
        gyr = d.sensordata[3:6]
        psi = d.qpos[self.qa_head]
        c, s = np.cos(psi), np.sin(psi)
        # head frame -> base-aligned axes (the firmware does this with q_head)
        fx = c * acc[0] - s * acc[1]
        fz = acc[2]
        gy = s * gyr[0] + c * gyr[1]
        gz = gyr[2]
        n_g = p.gyro_noise_dps
        pit_gyro = np.rad2deg(gy) + self.gyro_bias + rng.normal(0.0, n_g)
        pit_acc = np.rad2deg(np.arctan2(-(fx + p.acc_bias + rng.normal(0.0, p.acc_noise)),
                                       fz + rng.normal(0.0, p.acc_noise))) + p.pitch_offset_deg
        dt_imu = max(t - self.t_imu, 1e-3) if self.k > 0 else self.plant.imu_period
        self.t_imu = t
        pit_freq = 1.0 / (abs(pit_gyro) + 200.0)
        if abs(pit_acc) < 100.0 and abs(pit_gyro) < 1000.0:
            self.imu_pitch = pit_freq * pit_acc + (1.0 - pit_freq) * (self.imu_pitch + pit_gyro * dt_imu)
        self.imu_dq = 0.6 * self.imu_dq + 0.4 * pit_gyro
        yaw_raw = np.rad2deg(gz) + p.yaw_bias_dps + rng.normal(0.0, n_g)
        self.imu_yaw_rate = 0.5 * self.imu_yaw_rate + 0.5 * yaw_raw
        self.imu_hist.append((t, self.imu_pitch, self.imu_dq, self.imu_yaw_rate))

    def _imu_outputs(self, t):
        """Latest filter output that is at least imu_extra_delay old (I2C read + task hand-over latency)."""
        tt = t - self.plant.imu_extra_delay
        out = self.imu_hist[0]
        for item in self.imu_hist:
            if item[0] <= tt:
                out = item
            else:
                break
        return out

    def _wheel_events(self, t):
        p, d = self.plant, self.data
        for i, (da, qa) in enumerate(((self.da_wl, self.qa_wl), (self.da_wr, self.qa_wr))):
            if self.w_pending[i] is not None and t >= self.w_pending[i][0]:
                _, w, pos = self.w_pending[i]
                self.w_filt[i] = 0.7 * self.w_filt[i] + 0.3 * w
                self.w_pos_hold[i] = pos
                self.w_commit_t[i] = t
                self.w_pending[i] = None
            if t >= self.t_w[i]:
                w = p.speed_scale * float(d.qvel[da]) + self.rng.normal(0.0, p.speed_noise)
                pos = float(d.qpos[qa])
                self.w_pending[i] = (t + 0.004, w, np.round(pos / (2 * np.pi) * 1024) / 1024 * 2 * np.pi)
                self.t_w[i] += p.ctrl_dt * p.wheel_update_ticks

    # ------------------------------------------------------------------ observation / privileged
    def _observe(self):
        t = self.t
        _, pit, dq, yr = self._imu_outputs(t)
        self.v_meas = R_TYRE * 0.5 * (self.w_filt[0] + self.w_filt[1])      # unfiltered (firmware EMA only)
        o = np.empty(self.obs_dim, np.float32)
        o[0] = np.deg2rad(pit) - self._pitch_zero
        o[1] = np.deg2rad(dq)
        o[2] = np.deg2rad(yr)
        o[3] = self.v_obs
        o[4] = self.v_cmd
        o[5] = self.r_cmd
        o[6] = self.ev_int
        o[7] = self.er_int
        if self.obs_dim > 8:
            o[8] = self.ep_int
        self._obs_raw = o.copy()
        return np.clip(o * self.obs_scale, -5.0, 5.0)

    _pitch_zero = 0.0

    def _priv(self):
        o = self._obs_raw * self.obs_scale
        extra = np.array([
            self.pitch / 0.3, self.q / 3.0, self.v_true / 0.5, self.r_true / 3.0, self.roll / 0.2, self.roll_rate / 2.0,
            self.v_lat / 0.3, self.head_angle / 1.5, self.wl_rel / 30.0, self.wr_rel / 30.0,
            self._last_action[0], self._last_action[1], self._push_vec[0] / 1.5, self._push_vec[1] / 1.5,
            self.theta_eq / 0.15, np.sin(self.heading), np.cos(self.heading), self.pos_err / 0.3], dtype=np.float32)
        return np.concatenate([np.clip(o, -5, 5), extra, PL.privileged_vector(self.plant)]).astype(np.float32)

    _push_vec = (0.0, 0.0)

    # ------------------------------------------------------------------ stepping
    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        ac, ad = float(a[0]), float(a[1])
        cl = float(np.clip(ac - ad, -1.0, 1.0)) * CMD_UNITS
        cr = float(np.clip(ac + ad, -1.0, 1.0)) * CMD_UNITS
        obs, priv, rew, term, trunc, info = self.step_cmd(cl, cr)
        self._last_action = a.astype(np.float32)
        priv[self.obs_dim + 10:self.obs_dim + 12] = self._last_action
        return obs, priv, rew, term, trunc, info

    def step_cmd(self, cmd_l, cmd_r):
        """One control period with raw wheel commands (command units, +forward). Returns the same tuple as step()."""
        p, cfg, d, m = self.plant, self.cfg, self.data, self.model
        dt = self.dt
        # control period jitter: 2 ms physics steps, alternate between floor and ceil of the sampled period
        nominal = p.ctrl_dt / dt
        n_sub = int(nominal) + (1 if self.rng.random() < nominal - int(nominal) else 0)
        n_sub = max(n_sub, 2)
        self._set_commands(self.t)
        self._slew_commands(n_sub * dt)
        self.last_cmd[:] = (cmd_l, cmd_r)
        cmd_buf, nb, ds = self.cmd_buf, self.nbuf, self.delay_steps
        uf0, uf1 = self.uf
        al = self.alpha_lag
        ktl, ktr = self.kt
        dzl, dzr = p.deadzone_l, p.deadzone_r
        k = self.k
        ctrl = d.ctrl
        for _ in range(n_sub):
            k += 1
            t = self.t + dt
            self.t = t
            cmd_buf[0, k % nb] = cmd_l
            cmd_buf[1, k % nb] = cmd_r
            kd = (k - ds) % nb
            uf0 += al * (cmd_buf[0, kd] - uf0)
            uf1 += al * (cmd_buf[1, kd] - uf1)
            u0 = uf0 - dzl if uf0 > dzl else (uf0 + dzl if uf0 < -dzl else 0.0)
            u1 = uf1 - dzr if uf1 > dzr else (uf1 + dzr if uf1 < -dzr else 0.0)
            ctrl[0] = ktl * u0
            ctrl[1] = ktr * u1
            self._frictions()
            self._disturbances(t)
            mujoco.mj_step(m, d)
            if t >= self.t_imu + self._imu_next_gap:
                self._imu_sample(t)
                self._imu_next_gap = p.imu_period * (0.67 if self.rng.random() < 0.5 else 1.33)
            self._wheel_events(t)
        self.k = k
        self.uf = [uf0, uf1]
        self.n_ticks += 1
        self._update_truth()
        # integrators on measured quantities (firmware side)
        tick = n_sub * dt
        self._observe_update_integrators(tick)
        self.pos_err += (self.v_true - self.v_cmd) * tick
        self.pos_err -= self.pos_err * tick / cfg.x_leak_s
        self.pos_err = float(np.clip(self.pos_err, -0.5, 0.5))
        obs = self._observe()
        rew, term = self._reward()
        trunc = (self.t >= cfg.episode_s) and not term
        info = dict(ev=self.v_true - self.v_cmd, er=self.r_true - self.r_cmd, pitch=self.pitch, tick=tick,
                    vt=self.v_target, rt=self.r_target)
        return obs, self._priv(), rew, term, trunc, info

    _imu_next_gap = 0.003

    def _observe_update_integrators(self, tick):
        cfg = self.cfg
        _, pit, dq, yr = self._imu_outputs(self.t)
        v_meas = R_TYRE * 0.5 * (self.w_filt[0] + self.w_filt[1])
        self.v_obs += (v_meas - self.v_obs) * tick / (cfg.speed_lp_s + tick)
        r_meas = np.deg2rad(yr)
        self.ev_int += (self.v_obs - self.v_cmd) * tick
        self.ev_int -= self.ev_int * tick / cfg.ev_leak_s
        self.ev_int = float(np.clip(self.ev_int, -cfg.ev_clip, cfg.ev_clip))
        self.ep_int += self.ev_int * tick
        self.ep_int -= self.ep_int * tick / cfg.ep_leak_s
        self.ep_int = float(np.clip(self.ep_int, -cfg.ep_clip, cfg.ep_clip))
        self.er_int += (r_meas - self.r_cmd) * tick
        self.er_int -= self.er_int * tick / cfg.er_leak_s
        self.er_int = float(np.clip(self.er_int, -cfg.er_clip, cfg.er_clip))

    # friction (Coulomb + Stribeck stiction) re-evaluated each physics step from the wheel speeds
    def _frictions(self):
        p, d, m = self.plant, self.data, self.model
        if p.coulomb_l == 0.0 and p.coulomb_r == 0.0 and p.stiction_l == 0.0 and p.stiction_r == 0.0:
            return
        wl, wr = d.qvel[self.da_wl], d.qvel[self.da_wr]
        s = p.stribeck_speed
        m.dof_frictionloss[self.da_wl] = p.coulomb_l + p.stiction_l * np.exp(-(wl / s) ** 2)
        m.dof_frictionloss[self.da_wr] = p.coulomb_r + p.stiction_r * np.exp(-(wr / s) ** 2)

    def _disturbances(self, t):
        d, m, cfg, rng = self.data, self.model, self.cfg, self.rng
        # random pushes (force at a point on the robot), scripted pushes for evaluation
        if self.push is None:
            fire = None
            if t >= self.t_push_next:
                ang = rng.choice([0.0, np.pi, np.pi / 2, -np.pi / 2], p=[0.35, 0.35, 0.15, 0.15]) + rng.normal(0, 0.15)
                if cfg.push_log:
                    mag = float(np.exp(rng.uniform(np.log(cfg.push_force[0]), np.log(cfg.push_force[1])))) * self.plant.push_scale * (0.3 + 0.7 * self.domain)
                else:
                    mag = rng.uniform(*cfg.push_force) * self.plant.push_scale * (0.3 + 0.7 * self.domain)
                if abs(np.sin(ang)) > 0.7:
                    mag = min(mag, cfg.push_lat_max)
                fire = (t, mag * np.cos(ang), mag * np.sin(ang), rng.uniform(*cfg.push_h), rng.uniform(*cfg.push_dur))
                self.t_push_next = t + rng.uniform(*cfg.push_interval)
            elif self.d_push_script and t >= self.d_push_script[0][0]:
                fire = self.d_push_script.pop(0)
            if fire is not None:
                self.push = dict(t0=t, fx=fire[1], fy=fire[2], h=fire[3], dur=fire[4])
        if self.push is not None:
            pu = self.push
            if t - pu['t0'] > pu['dur']:
                self.push = None
                d.qfrc_applied[:] = 0.0
                self._push_vec = (0.0, 0.0)
            else:
                d.qfrc_applied[:] = 0.0
                R = d.xmat[self.b_base].reshape(3, 3)
                fw = R @ np.array([pu['fx'], pu['fy'], 0.0])       # force in the body-aligned frame
                point = d.xpos[self.b_base] + R @ np.array([0.0, 0.0, pu['h']])
                mujoco.mj_applyFT(m, d, fw, np.zeros(3), point, self.b_base, d.qfrc_applied)
                self._push_vec = (pu['fx'], pu['fy'])
        # head (EM3 servo) wobble: smooth random target, rate limited
        p = self.plant
        if p.head_wobble > 0.0:
            if t >= self.t_head_next:
                self.head_goal = float(np.clip(p.head_static + rng.uniform(-p.head_wobble, p.head_wobble), -2.4, 2.4))
                self.t_head_next = t + rng.uniform(0.4, 2.0)
            step = 3.0 * self.dt
            self.head_target += float(np.clip(self.head_goal - self.head_target, -step, step))
        else:
            step = 3.0 * self.dt
            self.head_target += float(np.clip(p.head_static - self.head_target, -step, step))
        d.ctrl[2] = self.head_target

    # ------------------------------------------------------------------ reward
    def _reward(self):
        cfg = self.cfg
        fell = abs(self.pitch) > cfg.fall_pitch or abs(self.roll) > cfg.fall_roll
        if fell:
            self.fell = True
            return -cfg.fall_penalty, True
        ev = self.v_true - self.v_cmd
        er = self.r_true - self.r_cmd
        r = cfg.w_alive
        # narrow kernel for precision + saturating linear penalty so large errors still have a gradient and a cost
        r += cfg.w_v * np.exp(-(ev / cfg.sig_v) ** 2) - cfg.w_v2 * min(abs(ev), cfg.v_sat) / cfg.v_sat
        r += cfg.w_r * np.exp(-(er / cfg.sig_r) ** 2) - cfg.w_r2 * min(abs(er), cfg.r_sat) / cfg.r_sat
        r -= cfg.w_x * min(abs(self.pos_err), cfg.x_sat) / cfg.x_sat
        r += cfg.w_xb * np.exp(-(self.pos_err / cfg.sig_x) ** 2)
        if cfg.w_turn_v > 0.0 and abs(self.v_cmd) < 0.02 and abs(self.r_cmd) > 0.3:
            r -= cfg.w_turn_v * min(abs(self.v_true), cfg.turn_v_sat) / cfg.turn_v_sat + cfg.w_turn_q * min(self.q ** 2, 4.0)
        r -= cfg.w_q * min((self.q / 0.5) ** 2, 4.0)
        r -= cfg.w_roll * (self.roll / 0.1) ** 2
        r -= cfg.w_pitch * ((self.pitch - self.theta_eq) / 0.15) ** 2
        return float(r), False

    def action_penalty(self, mean, prev_mean):
        """Smoothness / effort penalty on the policy's DETERMINISTIC output (not on the exploration noise)."""
        cfg = self.cfg
        d = np.asarray(mean) - np.asarray(prev_mean)
        return cfg.w_dact * float(d @ d) / 0.01 ** 2 + cfg.w_act * float(np.asarray(mean) @ np.asarray(mean)) / 0.3 ** 2

    # convenience for evaluation scripts ---------------------------------------------------------------
    @property
    def measured(self):
        _, pit, dq, yr = self._imu_outputs(self.t)
        return dict(pitch_deg=pit, dq_dps=dq, yaw_rate_dps=yr, wheel_w=tuple(self.w_filt), wheel_pos=tuple(self.w_pos_hold),
                    v_meas=R_TYRE * 0.5 * (self.w_filt[0] + self.w_filt[1]), v_obs=self.v_obs)


PL_WHEEL_ASSY = 0.1329
