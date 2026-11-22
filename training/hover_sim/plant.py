"""Actuator dynamics and domain randomization for Hover. Units: metres, seconds and newton-metres."""
from dataclasses import dataclass, asdict, field

import numpy as np

# nominal values (see docstring)
KT_NOM = 1.87e-4          # N.m per command unit
DAMPING_NOM = 2.0e-3      # N.m.s/rad
LAG_NOM = 0.0072          # s
DELAY_NOM = 0.004         # s (2 ms firmware compute->send + 2 ms driver)
CMD_MAX = 1000.0          # command units (firmware clips the LQR output at 1000)
TYRE_R = 0.0308


@dataclass
class Plant:
    """One sampled plant. Units: N.m, s, command units."""
    kt: float = KT_NOM
    damping: float = DAMPING_NOM
    delay: float = DELAY_NOM
    lag: float = LAG_NOM
    coulomb_l: float = 0.0
    coulomb_r: float = 0.0
    stiction_l: float = 0.0         # extra static friction (Stribeck, fades over `stribeck_speed`)
    stiction_r: float = 0.0
    stribeck_speed: float = 0.4     # rad/s
    deadzone_l: float = 0.0         # command units
    deadzone_r: float = 0.0
    kt_asym: float = 1.0            # right/left torque-constant ratio
    speed_scale: float = 1.0        # wheel-speed sensor gain error
    speed_noise: float = 0.08       # rad/s per wheel-speed update
    # IMU
    gyro_noise_dps: float = 0.25
    gyro_bias_dps: float = 0.0
    yaw_bias_dps: float = 0.0
    acc_noise: float = 0.05         # m/s^2
    acc_bias: float = 0.0           # m/s^2 (along the sensor x)
    pitch_offset_deg: float = 0.0   # IMU mounting / calibration error of the pitch zero
    imu_extra_delay: float = 0.001
    imu_period: float = 0.003       # mean sample period (s)
    # timing
    ctrl_dt: float = 0.0065
    wheel_update_ticks: int = 6
    # world
    mu: float = 0.9
    tyre_halfwidth: float = 0.0040   # contact patch half width (m): sets the yaw scrub of the tyres
    slope_deg: float = 0.0
    # body (applied in env._apply_body_randomisation)
    head_mass_scale: float = 1.0
    head_com: tuple = (0.0, 0.0, 0.0)       # extra CoM shift (m)
    base_com: tuple = (0.0, 0.0, 0.0)
    inertia_scale_head: float = 1.0
    inertia_scale_base: float = 1.0
    stator_share: float = 0.55
    total_mass_scale: float = 1.0
    # disturbances
    push_scale: float = 1.0
    head_wobble: float = 0.0        # amplitude (rad) of random head pan motion
    head_static: float = 0.0        # rad

    def as_dict(self):
        return asdict(self)


def sample(rng: np.random.Generator, domain: float = 1.0, corner: bool = False, nominal_only: bool = False) -> Plant:
    """Draw a plant. ``domain`` in [0, 1] scales every randomisation range; ``corner`` puts each parameter at
    one end of its range (those are the episodes where thin-margin controllers fail)."""
    p = Plant()
    if nominal_only:
        return p
    d = float(domain)

    def u(lo, hi):
        if corner and d > 0:
            return lo if rng.random() < 0.5 else hi
        return float(rng.uniform(lo, hi))

    def around(nom, lo_rel, hi_rel):
        # nominal * (1 + u(lo, hi) * d)
        return nom * (1.0 + u(lo_rel, hi_rel) * d)

    p.kt = around(KT_NOM, -0.30, 0.40)
    p.kt_asym = 1.0 + u(-0.08, 0.08) * d
    p.damping = max(5e-4, around(DAMPING_NOM, -0.5, 0.8))
    p.delay = max(0.002, DELAY_NOM + u(-0.001, 0.008) * d)
    p.lag = max(0.003, around(LAG_NOM, -0.4, 0.6))
    # friction / dead zone (the real wheels differ from robot to robot and from wheel to wheel)
    p.coulomb_l = u(0.0, 0.006) * d
    p.coulomb_r = u(0.0, 0.006) * d
    if corner or rng.random() < 0.5 * d:
        p.deadzone_l = u(0.0, 25.0) * d
        p.deadzone_r = u(0.0, 25.0) * d
    if rng.random() < 0.3 * d:
        p.stiction_l = u(0.0, 0.004) * d
        p.stiction_r = u(0.0, 0.004) * d
    p.speed_scale = around(1.0, -0.10, 0.10)
    p.speed_noise = 0.08 * (1.0 + u(0.0, 1.5) * d)
    # IMU
    p.gyro_noise_dps = 0.25 * (1.0 + u(0.0, 1.5) * d)
    p.gyro_bias_dps = u(-1.5, 1.5) * d
    p.yaw_bias_dps = u(-0.4, 0.4) * d
    p.acc_noise = 0.05 * (1.0 + u(0.0, 1.5) * d)
    p.acc_bias = u(-0.15, 0.15) * d
    p.pitch_offset_deg = u(-3.0, 3.0) * d
    p.imu_extra_delay = max(0.0, 0.001 + u(-0.001, 0.003) * d)
    p.imu_period = 0.003 + u(-0.0005, 0.001) * d
    p.ctrl_dt = 0.0065 + u(-0.0010, 0.0015) * d
    p.wheel_update_ticks = int(rng.integers(3, 8)) if d > 0 else 6
    # world
    p.mu = float(np.clip(0.9 + u(-0.35, 0.25) * d, 0.3, 1.3))
    p.slope_deg = u(-1.0, 1.0) * d            # a desk is level to within ~1 deg (the first contract used +-3 deg)
    p.tyre_halfwidth = 0.0040 + u(-0.0010, 0.0020) * d
    # body
    p.total_mass_scale = 1.0 + u(-0.015, 0.015) * d
    p.head_mass_scale = 1.0 + u(-0.15, 0.15) * d
    p.head_com = (u(-0.003, 0.003) * d, u(-0.002, 0.002) * d, u(-0.006, 0.006) * d)
    p.base_com = (u(-0.0015, 0.0015) * d, u(-0.001, 0.001) * d, u(-0.002, 0.002) * d)
    p.inertia_scale_head = 1.0 + u(-0.2, 0.25) * d
    p.inertia_scale_base = 1.0 + u(-0.2, 0.25) * d
    p.stator_share = float(np.clip(0.55 + u(-0.2, 0.2) * d, 0.35, 0.75))
    # head (EM3 servo) motion: static pan angle and/or random wobble
    if rng.random() < 0.5 * d:
        p.head_static = float(rng.uniform(-1.2, 1.2))
    if rng.random() < 0.3 * d:
        p.head_wobble = float(rng.uniform(0.2, 1.0))
    p.push_scale = 1.0
    return p


# vector used as extra critic input (privileged), normalised roughly to [-1, 1]
def privileged_vector(p: Plant):
    return np.array([
        (p.kt / KT_NOM - 1.0) / 0.4, (p.damping / DAMPING_NOM - 1.0) / 0.8, (p.delay - 0.008) / 0.006,
        (p.lag / LAG_NOM - 1.0) / 0.6, p.coulomb_l / 0.006, p.coulomb_r / 0.006,
        p.deadzone_l / 25.0, p.deadzone_r / 25.0, p.stiction_l / 0.004, (p.speed_scale - 1.0) / 0.10,
        p.pitch_offset_deg / 3.0, p.acc_bias / 0.15, p.gyro_bias_dps / 1.5, (p.mu - 0.9) / 0.3, p.slope_deg / 1.0,
        (p.head_mass_scale - 1.0) / 0.15, p.head_com[0] / 0.003, p.head_com[1] / 0.002, p.head_com[2] / 0.006,
        p.base_com[0] / 0.0015, (p.inertia_scale_head - 1.0) / 0.25, (p.inertia_scale_base - 1.0) / 0.25,
        (p.stator_share - 0.55) / 0.2, (p.ctrl_dt - 0.0065) / 0.0015, p.head_static / 1.2, p.head_wobble, (p.tyre_halfwidth - 0.004) / 0.002,
    ], dtype=np.float32)


PRIV_DIM = 27
