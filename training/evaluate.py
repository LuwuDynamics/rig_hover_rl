"""Evaluate a policy (or the firmware LQR baseline) on scripted scenarios in the randomised simulator.

    .venv/bin/python evaluate.py --policy runs/v1/actor.json --seeds 24
    .venv/bin/python evaluate.py --policy lqr --scenarios stand stand_push --seeds 24
    .venv/bin/python evaluate.py --policy runs/v1/actor.json --plot logs/eval_v1   # also write time-series PNGs

Every episode draws a fresh plant from the training randomisation (domain 1.0 unless a scenario says otherwise);
the seed list is fixed, so two policies are compared on identical robots.
"""
import os
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('MKL_NUM_THREADS', '1')
import argparse
import dataclasses
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from hover_sim.env import HoverEnv, make_config  # noqa: E402
from hover_sim.np_actor import NumpyActor  # noqa: E402
from hover_sim.baselines import FirmwareLQR  # noqa: E402
from hover_sim import plant as PL  # noqa: E402

DT_TICK = 0.0065


def step_profile(points):
    """Piecewise-constant (t, v, r) -> callable."""
    pts = sorted(points)

    def f(t):
        v = r = 0.0
        for (t0, vv, rr) in pts:
            if t >= t0:
                v, r = vv, rr
        return v, r
    return f


def random_pushes(rng, T, scale=1.0, first=2.0):
    out = []
    t = first
    while t < T - 1.0:
        ang = rng.choice([0.0, np.pi, np.pi / 2, -np.pi / 2], p=[0.35, 0.35, 0.15, 0.15]) + rng.normal(0, 0.15)
        mag = rng.uniform(0.1, 0.6) * scale
        out.append((t, mag * np.cos(ang), mag * np.sin(ang), rng.uniform(0.0, 0.10), rng.uniform(0.04, 0.10)))
        t += rng.uniform(1.5, 3.0)
    return out


# scenario -> dict(T, build(rng) -> (script|None), plant overrides, domain, corner)
# Commands are at the desktop-toy scale of the default contract (|v| <= 0.15 m/s, |r| <= 2 rad/s).
def scenarios():
    S = {}
    S['stand'] = dict(T=20.0, script=lambda rng: dict(cmd=lambda t: (0.0, 0.0)))
    S['stand_push'] = dict(T=30.0, script=lambda rng: dict(cmd=lambda t: (0.0, 0.0), pushes=random_pushes(rng, 30.0)))
    S['fwd_back'] = dict(T=26.0, script=lambda rng: dict(
        cmd=step_profile([(0, 0, 0), (2, 0.05, 0), (6, 0, 0), (8, -0.05, 0), (12, 0, 0), (14, 0.10, 0), (18, 0, 0), (20, -0.10, 0), (24, 0, 0)])))
    S['fwd_max'] = dict(T=22.0, script=lambda rng: dict(
        cmd=step_profile([(0, 0, 0), (2, 0.15, 0), (7, 0, 0), (9, -0.15, 0), (14, 0, 0), (16, 0.15, 0), (19, -0.15, 0)])))
    S['turn'] = dict(T=26.0, script=lambda rng: dict(
        cmd=step_profile([(0, 0, 0), (2, 0, 1.0), (6, 0, 0), (8, 0, -1.0), (12, 0, 0), (14, 0, 2.0), (18, 0, -2.0), (22, 0, 0)])))
    S['circle'] = dict(T=26.0, script=lambda rng: dict(
        cmd=step_profile([(0, 0, 0), (2, 0.06, 0.6), (9, 0.06, -0.6), (16, 0.10, 1.0), (22, 0, 0)])))
    S['drive'] = dict(T=40.0, script=None, push_prob=1.0)           # the training command generator + pushes
    stress_cmd = [(0, 0, 0), (3, 0.08, 0), (8, 0, 1.0), (13, -0.08, 0), (17, 0.06, 0.8)]
    stress = lambda rng: dict(cmd=step_profile(stress_cmd), pushes=random_pushes(rng, 22.0, 0.7))   # noqa: E731
    S['stress_gain_hi'] = dict(T=22.0, script=stress, plant=dict(kt_mult=1.7))
    S['stress_gain_lo'] = dict(T=22.0, script=stress, plant=dict(kt_mult=0.55))
    S['stress_delay'] = dict(T=22.0, script=stress, plant=dict(extra_delay=0.008))
    S['fwd_back_cal'] = dict(T=26.0, script=S['fwd_back']['script'], plant=dict(speed_scale=1.0))
    S['scale_lo'] = dict(T=22.0, script=stress, plant=dict(speed_scale=0.6))     # wheel-speed reading 40% too low
    S['scale_hi'] = dict(T=22.0, script=stress, plant=dict(speed_scale=1.5))     # 50% too high
    turn_script = S['turn']['script']
    S['turn_hw15'] = dict(T=26.0, script=turn_script, plant=dict(tyre_hw=0.0015))     # very low yaw scrub (1.5 mm effective contact half-width)
    S['turn_hw80'] = dict(T=26.0, script=turn_script, plant=dict(tyre_hw=0.0080))     # high scrub (flat 16 mm tyre)
    for grp in ('friction', 'asym', 'kt', 'imu', 'com', 'head', 'timing', 'scrub', 'slope'):
        S['turn_z_' + grp] = dict(T=26.0, script=turn_script, plant=dict(zero=[grp]))
    S['turn_z_all'] = dict(T=26.0, script=turn_script, plant=dict(zero=['friction', 'asym', 'kt', 'imu', 'com', 'head', 'timing', 'scrub', 'slope']))
    S['noise5'] = dict(T=22.0, script=lambda rng: dict(cmd=step_profile(stress_cmd)), plant=dict(noise_mult=5.0))
    S['corner'] = dict(T=22.0, script=stress, corner=True)
    S['nominal'] = dict(T=22.0, script=lambda rng: dict(cmd=step_profile([(0, 0, 0), (3, 0.08, 0), (8, 0, 1.5), (13, -0.08, 0), (17, 0, 0)])),
                        domain=0.0)
    return S


def hp_rms(x, tick=DT_TICK, fc=3.0):
    """RMS of x above ~fc Hz (x minus its first-order low-pass)."""
    a = tick / (tick + 1.0 / (2 * np.pi * fc))
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc += a * (v - acc)
        y[i] = acc
    return float(np.sqrt(np.mean((x - y) ** 2)))


def apply_overrides(plant, spec, rng):
    if not spec:
        return plant
    if 'kt_mult' in spec:
        plant.kt *= spec['kt_mult']
    if 'extra_delay' in spec:
        plant.delay += spec['extra_delay']
    for grp in spec.get('zero', ()):      # attribution experiments: neutralise one group of randomised parameters
        if grp == 'friction':
            plant.coulomb_l = plant.coulomb_r = plant.deadzone_l = plant.deadzone_r = plant.stiction_l = plant.stiction_r = 0.0
        elif grp == 'asym':
            plant.kt_asym = 1.0
        elif grp == 'kt':
            plant.kt = PL.KT_NOM
        elif grp == 'imu':
            plant.pitch_offset_deg = plant.gyro_bias_dps = plant.acc_bias = plant.yaw_bias_dps = 0.0
        elif grp == 'com':
            plant.head_com = plant.base_com = (0.0, 0.0, 0.0)
            plant.head_mass_scale = plant.inertia_scale_head = plant.inertia_scale_base = 1.0
        elif grp == 'head':
            plant.head_static = plant.head_wobble = 0.0
        elif grp == 'timing':
            plant.wheel_update_ticks, plant.ctrl_dt, plant.speed_scale = 6, 0.0065, 1.0
        elif grp == 'scrub':
            plant.tyre_halfwidth = 0.004
        elif grp == 'slope':
            plant.slope_deg = 0.0
    if 'tyre_hw' in spec:
        plant.tyre_halfwidth = spec['tyre_hw']
    if 'speed_scale' in spec:
        plant.speed_scale = spec['speed_scale']
    if 'noise_mult' in spec:
        m = spec['noise_mult']
        plant.gyro_noise_dps *= m
        plant.acc_noise *= m
        plant.speed_noise *= m
    return plant


def run_episode(args):
    name, spec, policy_path, seed, record = args
    sc = scenarios()[name]
    cfg = make_config()
    cfg.episode_s = sc['T']
    domain = sc.get('domain', 1.0)
    env = HoverEnv(seed=seed, domain=domain, config=cfg, push_prob=sc.get('push_prob', 0.0), corner_prob=0.0)
    rng = np.random.default_rng(seed + 555)
    script = sc['script'](rng) if sc['script'] else None
    plant = PL.sample(np.random.default_rng(seed + 31337), domain, corner=sc.get('corner', False))
    plant.wheel_update_ticks = 6 if policy_path == 'lqr' else plant.wheel_update_ticks
    plant = apply_overrides(plant, sc.get('plant'), rng)
    obs, priv = env.reset(plant=plant, script=script, seed=seed + 99)
    if sc['script'] is None:
        env.cfg.episode_s = sc['T']
    if policy_path == 'lqr':
        ctrl = FirmwareLQR()
        ctrl.reset()
        act = None
    else:
        actor = NumpyActor.from_json(policy_path)
        act = lambda o: actor.mean(o)  # noqa: E731
    rows = []
    term = trunc = False
    cmd_c = []
    while not (term or trunc):
        if policy_path == 'lqr':
            cl, cr = ctrl(env, vx_cmd_units=0.0)
            obs, priv, rew, term, trunc, info = env.step_cmd(cl, cr)
            cc = 0.5 * (cl + cr)
        else:
            a = act(obs)
            obs, priv, rew, term, trunc, info = env.step(a)
            cc = float(np.clip(a[0], -1, 1)) * 1000.0
        cmd_c.append(cc)
        if record:
            rows.append((env.t, env.pitch, env.q, env.v_true, env.v_cmd, env.r_true, env.r_cmd, cc, env.roll,
                         env.last_cmd[0], env.last_cmd[1], env.v_meas, env.data.qpos[0], env.data.qpos[1]))
    n = len(cmd_c)
    rows_np = np.array(rows) if record else None
    # metrics (skip the first 1.5 s)
    out = dict(name=name, seed=seed, fell=bool(env.fell), t=float(env.t))
    out['_hf'] = hp_rms(np.array(cmd_c))
    out['_cmdrms'] = float(np.sqrt(np.mean(np.square(cmd_c))))
    return out, rows_np, dict(ev2=0.0), env


def metrics_episode(args):
    """Run an episode and compute tracking metrics (needs the time series, always recorded)."""
    out, rows, _, env = run_episode(args[:4] + (True,))
    t = rows[:, 0]
    m = t > 1.5
    ev = rows[:, 3] - rows[:, 4]
    er = rows[:, 5] - rows[:, 6]
    out.update(v_rms=float(np.sqrt(np.mean(ev[m] ** 2))) if m.any() else np.nan,
               r_rms=float(np.sqrt(np.mean(er[m] ** 2))) if m.any() else np.nan,
               pitch_sd=float(np.rad2deg(np.std(rows[m, 1]))) if m.any() else np.nan,
               roll_max=float(np.rad2deg(np.abs(rows[:, 8]).max())),
               sat=float(np.mean((np.abs(rows[:, 9]) > 990) | (np.abs(rows[:, 10]) > 990))))
    k = int(1.5 / DT_TICK)
    if len(t) > 2 * k:
        const = (np.abs(rows[k:, 4] - rows[:-k, 4]) < 0.01) & (np.abs(rows[k:, 6] - rows[:-k, 6]) < 0.05) & (t[k:] > 2.0)
        if const.sum() > 50:
            out['v_ss'] = float(np.mean(np.abs(ev[k:][const])))
            out['r_ss'] = float(np.mean(np.abs(er[k:][const])))
    def extent(mask):
        """size of the world-frame bounding box of the axle trajectory (m -> mm)"""
        xy = rows[mask][:, 12:14]
        return 1000 * float(np.hypot(np.ptp(xy[:, 0]), np.ptp(xy[:, 1]))) if len(xy) else float('nan')

    if args[0].startswith('turn'):
        segs = [(2.6, 6.0), (8.6, 12.0), (14.6, 18.0), (18.6, 22.0)]
        vals = [extent((t > a) & (t < b)) for a, b in segs]
        vals = [v for v in vals if not np.isnan(v)]
        if vals:
            out['turn_p2p'] = float(np.mean(vals))
            out['turn_vmax'] = float(np.mean([np.abs(rows[(t > a) & (t < b), 3]).max() for a, b in segs if ((t > a) & (t < b)).any()]))
    # stationary position wander over the last 10 s of the stand scenarios: integrate v_true
    if args[0] in ('stand', 'stand_push'):
        out['wander_mm'] = extent(t > t[-1] - 10.0)
    return out, rows if args[4] else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', required=True, help="actor.json or 'lqr'")
    ap.add_argument('--scenarios', nargs='*', default=None)
    ap.add_argument('--seeds', type=int, default=24)
    ap.add_argument('--seed0', type=int, default=5000)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--json', default=None)
    ap.add_argument('--plot', default=None, help='directory for example time-series PNGs')
    args = ap.parse_args()
    names = args.scenarios or list(scenarios().keys())
    jobs = [(n, None, args.policy, args.seed0 + s, False) for n in names for s in range(args.seeds)]
    t0 = time.time()
    ctx = mp.get_context('spawn')
    with ctx.Pool(args.workers) as pool:
        results = pool.map(metrics_episode, jobs, chunksize=2)
    summary = {}
    print(f'policy: {args.policy}   seeds {args.seeds} ({time.time() - t0:.0f}s)')
    hdr = f"{'scenario':<14}{'survive':>9}{'v_rms':>8}{'r_rms':>8}{'v_ss':>7}{'r_ss':>7}{'pitch_sd':>9}{'cmd_hf':>8}{'cmd_rms':>8}{'roll_max':>9}{'sat%':>6}{'wander':>8}{'turn_p2p':>9}"
    print(hdr)
    for n in names:
        rs = [r for (r, _) in results if r['name'] == n]
        ok = [r for r in rs if not r['fell']]
        f = lambda k: float(np.nanmean([r[k] for r in rs if k in r])) if any(k in r for r in rs) else float('nan')  # noqa: E731
        summary[n] = dict(survive=len(ok) / len(rs), v_rms=f('v_rms'), r_rms=f('r_rms'), v_ss=f('v_ss'), r_ss=f('r_ss'), pitch_sd=f('pitch_sd'),
                          cmd_hf=f('_hf'), cmd_rms=f('_cmdrms'), roll_max=f('roll_max'), sat=f('sat') * 100,
                          wander_mm=f('wander_mm') if n in ('stand', 'stand_push') else float('nan'),
                          turn_p2p=f('turn_p2p'), turn_vmax=f('turn_vmax'), n=len(rs))
        s = summary[n]
        print(f"{n:<14}{len(ok):>5}/{len(rs):<3}{s['v_rms']:>8.3f}{s['r_rms']:>8.3f}{s['v_ss']:>7.3f}{s['r_ss']:>7.3f}{s['pitch_sd']:>9.2f}{s['cmd_hf']:>8.1f}{s['cmd_rms']:>8.1f}"
              f"{s['roll_max']:>9.1f}{s['sat']:>6.1f}{s['wander_mm']:>8.1f}{s['turn_p2p']:>9.0f}")
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(summary, indent=1))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        Path(args.plot).mkdir(parents=True, exist_ok=True)
        for n in names:
            out, rows = metrics_episode((n, None, args.policy, args.seed0, True))
            fig, ax = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
            ax[0].plot(rows[:, 0], rows[:, 3], label='v true'); ax[0].plot(rows[:, 0], rows[:, 4], '--', label='v cmd')
            ax[0].plot(rows[:, 0], rows[:, 11], ':', label='v measured'); ax[0].set_ylabel('m/s'); ax[0].legend(loc='upper right', ncol=3)
            ax[1].plot(rows[:, 0], rows[:, 5], label='yaw rate'); ax[1].plot(rows[:, 0], rows[:, 6], '--', label='cmd'); ax[1].set_ylabel('rad/s'); ax[1].legend(loc='upper right')
            ax[2].plot(rows[:, 0], np.rad2deg(rows[:, 1])); ax[2].set_ylabel('pitch deg')
            ax[3].plot(rows[:, 0], rows[:, 9], label='left'); ax[3].plot(rows[:, 0], rows[:, 10], label='right'); ax[3].set_ylabel('wheel cmd'); ax[3].set_xlabel('s'); ax[3].legend(loc='upper right')
            fig.suptitle(f"{n}  seed {args.seed0}  {'FELL' if out['fell'] else 'ok'}")
            fig.tight_layout()
            fig.savefig(Path(args.plot) / f'{n}.png', dpi=90)
            plt.close(fig)


if __name__ == '__main__':
    main()
