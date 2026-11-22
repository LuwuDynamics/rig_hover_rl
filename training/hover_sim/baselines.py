"""Baseline controllers for comparison: the LQR currently in the Hover firmware (robot.cc, UpdateWheelTorque)."""
import numpy as np


class FirmwareLQR:
    """u = -(1300 x + 5 v + 25 pitch_err + 0.4 dq), dead-zone kick of +-10 beyond +-20, clip 1000, yaw hold k=4.

    Units as in the firmware: x in m (wheel odometry with D = 60 mm), v in the firmware speed units
    (4.5 * K_V * register ~= 0.25 x the true mm/s of the Rider2-calibrated register), pitch in degrees, dq in deg/s.
    Physical forward command = +(...) for both wheels (tor1 = -u, tor2 = +u with wheel 2 mirrored).
    """

    def __init__(self, kx=1300.0, kv=5.0, kth=25.0, kdq=0.4, kyaw=4.0, pitch_ref_deg=0.0, v_unit_scale=0.25, yaw_clip=80.0):
        self.k = (kx, kv, kth, kdq)
        self.kyaw = kyaw
        self.ref = pitch_ref_deg
        self.vs = v_unit_scale
        self.yaw_clip = yaw_clip
        self.reset()

    def reset(self):
        self.x0 = None
        self.yaw_deg = 0.0
        self.yaw_ref = None
        self.t_last = None

    def __call__(self, env, vx_cmd_units=0.0):
        m = env.measured
        pos = 0.5 * (m['wheel_pos'][0] + m['wheel_pos'][1])           # rad (quantised, held)
        x_m = pos / (2 * np.pi) * np.pi * 0.060                         # firmware odometry: D = 60 mm
        if self.x0 is None:
            self.x0 = x_m
            self.t_last = env.t
        x = x_m - self.x0
        v_units = self.vs * m['v_meas'] * 1000.0
        kx, kv, kth, kdq = self.k
        c = kx * x + kv * (v_units - vx_cmd_units) + kth * (m['pitch_deg'] - self.ref) + kdq * m['dq_dps']
        if c > 20.0:
            c += 10.0
        elif c < -20.0:
            c -= 10.0
        c = float(np.clip(c, -1000.0, 1000.0))
        dt = env.t - self.t_last
        self.t_last = env.t
        self.yaw_deg += m['yaw_rate_dps'] * dt
        if self.yaw_ref is None:
            self.yaw_ref = self.yaw_deg
        diff = float(np.clip(-self.kyaw * (self.yaw_deg - self.yaw_ref), -self.yaw_clip, self.yaw_clip))
        return c - diff, c + diff          # left, right command units
