"""Average the actor weights of several checkpoints (Polyak-style weight averaging: usually smoother, never worse).

    .venv/bin/python tools/average_checkpoints.py runs/v2/ckpt-0*.pt --last 6 --out runs/v2/avg
Writes <out>.json (actor.json format) and <out>.pt (actor + critic of the last checkpoint).
"""
import argparse
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from hover_sim.policy import Actor, export_json  # noqa: E402
from hover_sim.env import OBS_SCALE_FAST, OBS_SCALE_TOY8, OBS_SCALE_TOY  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ckpts', nargs='+')
    ap.add_argument('--last', type=int, default=6)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    files = sorted(args.ckpts)[-args.last:]
    states = [torch.load(f, map_location='cpu')['actor'] for f in files]
    avg = {k: sum(s[k] for s in states) / len(states) for k in states[0]}
    dim = avg['body.0.weight'].shape[1]
    actor = Actor(obs_dim=dim)
    actor.load_state_dict(avg)
    scale = {9: OBS_SCALE_TOY, 8: OBS_SCALE_TOY8}[dim]
    last = torch.load(files[-1], map_location='cpu')
    out = Path(args.out)
    export_json(actor, str(out) + '.json', extra=dict(trained_steps=last.get('steps'), averaged_from=[Path(f).name for f in files]), obs_scale=scale)
    torch.save(dict(actor=avg, critic=last['critic'], steps=last.get('steps')), str(out) + '.pt')
    print('averaged', [Path(f).name for f in files], '->', str(out) + '.json')


if __name__ == '__main__':
    main()
