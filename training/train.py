"""Parallel PPO (asymmetric actor-critic) for the Hover robot.

    .venv/bin/python train.py --name v1 --steps 40e6 --workers 9

The actor sees only the 9 firmware-side observations; the critic additionally sees privileged simulator state
(true attitude/velocity, push force, every sampled plant parameter).
"""
import os
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('MKL_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('VECLIB_MAXIMUM_THREADS', '1')
import argparse
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from hover_sim.env import HoverEnv, EnvConfig, OBS_DIM, ACT_DIM  # noqa: E402
from hover_sim.np_actor import NumpyActor  # noqa: E402

LOG2PI = float(np.log(2 * np.pi))


def worker_main(conn, wid, n_envs, base_seed, env_kwargs):
    envs = [HoverEnv(seed=base_seed + 1000 * wid + i, **env_kwargs) for i in range(n_envs)]
    rng = np.random.default_rng(base_seed + 77 * wid + 1)
    states = [e.reset() for e in envs]
    prev_mean = np.zeros((n_envs, ACT_DIM))
    priv_dim = envs[0].priv_dim
    obs_dim = envs[0].obs_dim
    while True:
        msg = conn.recv()
        if msg['cmd'] == 'stop':
            break
        actor = NumpyActor(msg['weights'])
        log_std = msg['log_std']
        std = np.exp(log_std)
        T = msg['T']
        for e in envs:
            e.domain = msg['domain']
            e.cmd_scale = msg['cmd_scale']
        O = np.zeros((T, n_envs, obs_dim), np.float32)
        P = np.zeros((T, n_envs, priv_dim), np.float32)
        A = np.zeros((T, n_envs, ACT_DIM), np.float32)
        LP = np.zeros((T, n_envs), np.float32)
        R = np.zeros((T, n_envs), np.float32)
        TERM = np.zeros((T, n_envs), np.bool_)
        TRUNC = np.zeros((T, n_envs), np.bool_)
        FP = np.zeros((T, n_envs, priv_dim), np.float32)
        st = dict(falls=0, eps=0, len_sum=0, ev2=0.0, er2=0.0, n=0, ep_steps=np.zeros(n_envs))
        ep_len = msg.get('_ep_len')
        for t in range(T):
            ob = np.stack([s[0] for s in states])
            pr = np.stack([s[1] for s in states])
            mean = actor.mean(ob)
            eps = rng.standard_normal(mean.shape)
            a = mean + std * eps
            logp = -0.5 * (eps ** 2).sum(1) - log_std.sum() - 0.5 * ACT_DIM * LOG2PI
            O[t] = ob
            P[t] = pr
            A[t] = a
            LP[t] = logp
            for i, e in enumerate(envs):
                o2, p2, r, term, trunc, info = e.step(a[i])
                r -= e.action_penalty(mean[i], prev_mean[i])
                prev_mean[i] = mean[i]
                R[t, i] = r
                st['ev2'] += info['ev'] ** 2
                st['er2'] += info['er'] ** 2
                st['n'] += 1
                st['ep_steps'][i] += 1
                if term or trunc:
                    TERM[t, i] = term
                    TRUNC[t, i] = trunc
                    st['eps'] += 1
                    st['falls'] += int(term)
                    st['len_sum'] += st['ep_steps'][i]
                    st['ep_steps'][i] = 0
                    if trunc:
                        FP[t, i] = p2
                    states[i] = e.reset()
                    prev_mean[i] = 0.0
                else:
                    states[i] = (o2, p2)
        st['ep_steps'] = None
        conn.send(dict(O=O, P=P, A=A, LP=LP, R=R, TERM=TERM, TRUNC=TRUNC, FP=FP,
                       last_priv=np.stack([s[1] for s in states]), stats=st))
    conn.close()


def main():
    import torch
    import torch.nn as nn
    from hover_sim.policy import Actor, Critic, export_json

    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='run')
    ap.add_argument('--model', help='path to a MuJoCo XML model (default: bundled hover_dyn.xml)')
    ap.add_argument('--steps', type=float, default=40e6)
    ap.add_argument('--workers', type=int, default=9)
    ap.add_argument('--envs-per-worker', type=int, default=4)
    ap.add_argument('--horizon', type=int, default=512, help='steps per env per iteration')
    ap.add_argument('--epochs', type=int, default=6)
    ap.add_argument('--minibatch', type=int, default=8192)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.995)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.2)
    ap.add_argument('--target-kl', type=float, default=0.012)
    ap.add_argument('--init-std', type=float, default=0.10)
    ap.add_argument('--domain-start', type=float, default=0.3)
    ap.add_argument('--domain-warmup', type=float, default=0.30, help='fraction of training to reach domain 1')
    ap.add_argument('--cmd-start', type=float, default=0.4)
    ap.add_argument('--cmd-warmup', type=float, default=0.40)
    ap.add_argument('--corner-prob', type=float, default=0.3)
    ap.add_argument('--push-max', type=float, default=0.6, help='max random push force (N)')
    ap.add_argument('--push-prob', type=float, default=0.6)
    ap.add_argument('--push-dur-max', type=float, default=None, help='max push duration [s] (default 0.10)')
    ap.add_argument('--push-h-max', type=float, default=None, help='max height of the pushed point above the axle [m] (default 0.10)')
    ap.add_argument('--push-log', action='store_true', help='log-uniform push force')
    ap.add_argument('--push-lat-max', type=float, default=None, help='cap [N] for the sideways pushes')
    ap.add_argument('--fall-pitch', type=float, default=None, help='episode ends beyond this pitch/roll [rad] (default 0.8)')
    ap.add_argument('--w-x', type=float, default=None, help='position-hold reward weight (default: EnvConfig)')
    ap.add_argument('--x-leak', type=float, default=None)
    ap.add_argument('--w-turn-v', type=float, default=None)
    ap.add_argument('--w-turn-q', type=float, default=None)
    ap.add_argument('--resume', default=None)
    ap.add_argument('--resume-actor-only', action='store_true', help='new critic (different privileged input / reward)')
    ap.add_argument('--critic-warmup', type=int, default=0, help='iterations with the actor frozen while a fresh critic learns')
    ap.add_argument('--reset-std', type=float, default=0.0, help='re-initialise the exploration std after resuming')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--threads', type=int, default=4)
    ap.add_argument('--save-every', type=int, default=20)
    ap.add_argument('--reward-scale', type=float, default=0.1)
    args = ap.parse_args()
    torch.set_num_threads(args.threads)
    torch.manual_seed(args.seed)
    run_dir = ROOT / 'runs' / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / 'config.json').write_text(json.dumps(vars(args), indent=1))

    cfg = EnvConfig()
    cfg.push_force = (0.05, args.push_max)
    if args.push_dur_max is not None:
        cfg.push_dur = (0.04, args.push_dur_max)
    if args.push_h_max is not None:
        cfg.push_h = (0.0, args.push_h_max)
    cfg.push_log = args.push_log
    if args.push_lat_max is not None:
        cfg.push_lat_max = args.push_lat_max
    if args.fall_pitch is not None:
        cfg.fall_pitch = cfg.fall_roll = args.fall_pitch
    for name, val in (('w_x', args.w_x), ('x_leak_s', args.x_leak), ('w_turn_v', args.w_turn_v), ('w_turn_q', args.w_turn_q)):
        if val is not None:
            setattr(cfg, name, val)
    actor = Actor(obs_dim=cfg.n_obs, init_std=args.init_std)
    critic = Critic(HoverEnv.priv_dim_for(cfg))
    if args.resume:
        ck = torch.load(args.resume, map_location='cpu')
        actor.load_state_dict(ck['actor'])
        if not args.resume_actor_only:
            critic.load_state_dict(ck['critic'])
        if args.reset_std > 0:
            actor.log_std.data[:] = float(np.log(args.reset_std))
        print('resumed from', args.resume, '(actor only)' if args.resume_actor_only else '', flush=True)
    opt_a = torch.optim.Adam(actor.parameters(), lr=args.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=args.lr)
    lr = args.lr

    env_kwargs = dict(corner_prob=args.corner_prob, push_prob=args.push_prob, config=cfg)
    if args.model:
        env_kwargs['model_path'] = args.model
    ctx = mp.get_context('spawn')
    conns, procs = [], []
    for w in range(args.workers):
        a, b = ctx.Pipe()
        p = ctx.Process(target=worker_main, args=(b, w, args.envs_per_worker, args.seed * 100000 + 12345, env_kwargs), daemon=True)
        p.start()
        conns.append(a)
        procs.append(p)
    n_env = args.workers * args.envs_per_worker
    batch = n_env * args.horizon
    n_iter = int(args.steps // batch)
    print(f'{n_env} envs, {batch} steps/iteration, {n_iter} iterations', flush=True)

    ret_mean, ret_std = 0.0, 1.0
    t_start = time.time()
    total = 0
    log = open(run_dir / 'log.csv', 'w')
    log.write('iter,steps,reward,ep_len,fall_per_10k,ev_rms,er_rms,std_c,std_d,kl,lr,domain,cmd,fps\n')
    best = -1e9
    try:
        for it in range(n_iter):
            frac = it / max(n_iter - 1, 1)
            domain = min(1.0, args.domain_start + (1 - args.domain_start) * frac / max(args.domain_warmup, 1e-6))
            cmd_scale = min(1.0, args.cmd_start + (1 - args.cmd_start) * frac / max(args.cmd_warmup, 1e-6))
            weights, log_std = actor.numpy_weights()
            t0 = time.time()
            for c in conns:
                c.send(dict(cmd='rollout', weights=weights, log_std=log_std, T=args.horizon, domain=domain, cmd_scale=cmd_scale))
            res = [c.recv() for c in conns]
            t_roll = time.time() - t0
            O = np.concatenate([r['O'] for r in res], axis=1)
            P = np.concatenate([r['P'] for r in res], axis=1)
            A = np.concatenate([r['A'] for r in res], axis=1)
            LP = np.concatenate([r['LP'] for r in res], axis=1)
            R = np.concatenate([r['R'] for r in res], axis=1) * args.reward_scale
            TERM = np.concatenate([r['TERM'] for r in res], axis=1)
            TRUNC = np.concatenate([r['TRUNC'] for r in res], axis=1)
            FP = np.concatenate([r['FP'] for r in res], axis=1)
            LASTP = np.concatenate([r['last_priv'] for r in res], axis=0)
            T, N = R.shape
            with torch.no_grad():
                V = critic(torch.from_numpy(P.reshape(T * N, -1))).numpy().reshape(T, N) * ret_std + ret_mean
                Vl = critic(torch.from_numpy(LASTP)).numpy() * ret_std + ret_mean
                Vt = critic(torch.from_numpy(FP.reshape(T * N, -1))).numpy().reshape(T, N) * ret_std + ret_mean
            adv = np.zeros((T, N), np.float32)
            last = np.zeros(N, np.float32)
            for t in reversed(range(T)):
                nv = Vl if t == T - 1 else V[t + 1]
                nv = np.where(TRUNC[t], Vt[t], nv)
                nv = np.where(TERM[t], 0.0, nv)
                done = (TERM[t] | TRUNC[t])
                delta = R[t] + args.gamma * nv - V[t]
                last = delta + args.gamma * args.lam * np.where(done, 0.0, last)
                adv[t] = last
            ret = adv + V
            ret_mean = 0.9 * ret_mean + 0.1 * float(ret.mean())
            ret_std = 0.9 * ret_std + 0.1 * float(ret.std() + 1e-6)
            # ---- update
            ob_t = torch.from_numpy(O.reshape(T * N, -1))
            pr_t = torch.from_numpy(P.reshape(T * N, -1))
            a_t = torch.from_numpy(A.reshape(T * N, -1))
            lp_t = torch.from_numpy(LP.reshape(-1))
            adv_t = torch.from_numpy(adv.reshape(-1))
            adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)
            ret_t = torch.from_numpy(((ret - ret_mean) / ret_std).reshape(-1).astype(np.float32))
            n_tot = T * N
            kls = []
            stop = False
            for ep in range(args.epochs):
                perm = torch.randperm(n_tot)
                for s in range(0, n_tot, args.minibatch):
                    idx = perm[s:s + args.minibatch]
                    mean, ls = actor.dist_params(ob_t[idx])
                    std = ls.exp()
                    logp = (-0.5 * ((a_t[idx] - mean) / std) ** 2 - ls - 0.5 * LOG2PI).sum(-1)
                    ratio = (logp - lp_t[idx]).exp()
                    pg = -torch.min(ratio * adv_t[idx], ratio.clamp(1 - args.clip, 1 + args.clip) * adv_t[idx]).mean()
                    if it >= args.critic_warmup:
                        opt_a.zero_grad()
                        pg.backward()
                        nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
                        opt_a.step()
                    v = critic(pr_t[idx])
                    vl = 0.5 * ((v - ret_t[idx]) ** 2).mean()
                    opt_c.zero_grad()
                    vl.backward()
                    nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
                    opt_c.step()
                    with torch.no_grad():
                        kls.append(float(((lp_t[idx] - logp)).mean()))
                if np.mean(kls[-4:]) > 2.0 * args.target_kl:
                    stop = True
                if stop:
                    break
            kl = float(np.mean(kls))
            if kl > 2.0 * args.target_kl:
                lr = max(lr / 1.3, 3e-5)
            elif kl < 0.5 * args.target_kl:
                lr = min(lr * 1.2, 4e-4)
            for g in opt_a.param_groups + opt_c.param_groups:
                g['lr'] = lr
            total += n_tot
            # ---- stats
            eps_n = sum(r['stats']['eps'] for r in res)
            falls = sum(r['stats']['falls'] for r in res)
            len_sum = sum(r['stats']['len_sum'] for r in res)
            nstep = sum(r['stats']['n'] for r in res)
            ev = np.sqrt(sum(r['stats']['ev2'] for r in res) / nstep)
            er = np.sqrt(sum(r['stats']['er2'] for r in res) / nstep)
            rew = float(R.mean() / args.reward_scale)
            ep_len = len_sum / max(eps_n, 1)
            fall_rate = falls / max(nstep, 1) * 1e4
            sc, sd = np.exp(log_std)
            fps = n_tot / (time.time() - t0)
            line = (f'it {it:4d} steps {total / 1e6:6.2f}M rew/step {rew:6.3f} ep_len {ep_len:7.0f} falls/10k {fall_rate:5.1f} '
                    f'ev {ev:.3f} er {er:.3f} std {sc:.3f}/{sd:.3f} kl {kl:.4f} lr {lr:.1e} dom {domain:.2f} cmd {cmd_scale:.2f} '
                    f'| {fps:6.0f} fps (rollout {t_roll:.1f}s)')
            print(line, flush=True)
            log.write(f'{it},{total},{rew:.4f},{ep_len:.1f},{fall_rate:.4f},{ev:.4f},{er:.4f},{sc:.4f},{sd:.4f},{kl:.5f},{lr:.2e},{domain:.3f},{cmd_scale:.3f},{fps:.0f}\n')
            log.flush()
            if (it + 1) % args.save_every == 0 or it == n_iter - 1:
                ck = dict(actor=actor.state_dict(), critic=critic.state_dict(), it=it, steps=total, domain=domain, cmd=cmd_scale)
                torch.save(ck, run_dir / 'latest.pt')
                torch.save(ck, run_dir / f'ckpt-{it + 1:05d}.pt')
                export_json(actor, run_dir / 'actor.json', extra=dict(trained_steps=total, contract=cfg.contract), obs_scale=cfg.obs_scale)
    except KeyboardInterrupt:
        print('interrupted, saving', flush=True)
        ck = dict(actor=actor.state_dict(), critic=critic.state_dict(), it=it, steps=total)
        torch.save(ck, run_dir / 'latest.pt')
    finally:
        for c in conns:
            try:
                c.send(dict(cmd='stop'))
            except Exception:
                pass
        for p in procs:
            p.join(timeout=3)
        log.close()
    print('done in %.0f s' % (time.time() - t_start), flush=True)


if __name__ == '__main__':
    main()
