"""Actor / critic networks and the numpy twin used by the rollout workers and the exporter.

Deployed network (what the ESP32 runs):  obs(8, fixed scaling) -> Linear(32) -> tanh -> Linear(32) -> tanh -> Linear(2)
The critic is only used in training and also sees privileged simulator state.
"""
import json

import numpy as np
import torch
import torch.nn as nn

from .env import OBS_DIM, ACT_DIM, OBS_SCALE
from .np_actor import NumpyActor  # noqa: F401  (re-export)

HIDDEN = (32, 32)
LOG_STD_MIN, LOG_STD_MAX = np.log(0.01), np.log(0.6)


class Actor(nn.Module):
    def __init__(self, obs_dim=OBS_DIM, act_dim=ACT_DIM, hidden=HIDDEN, init_std=0.15):
        super().__init__()
        dims = [obs_dim, *hidden]
        layers = []
        for a, b in zip(dims[:-1], dims[1:]):
            layers += [nn.Linear(a, b), nn.Tanh()]
        self.body = nn.Sequential(*layers)
        self.head = nn.Linear(dims[-1], act_dim)
        self.log_std = nn.Parameter(torch.full((act_dim,), float(np.log(init_std))))
        for m in self.body:
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=1.0)
                nn.init.zeros_(m.bias)
        nn.init.orthogonal_(self.head.weight, gain=0.01)
        nn.init.zeros_(self.head.bias)

    def mean(self, obs):
        return self.head(self.body(obs))

    def dist_params(self, obs):
        return self.mean(obs), self.log_std.clamp(LOG_STD_MIN, LOG_STD_MAX)

    def numpy_weights(self):
        ws = []
        for m in self.body:
            if isinstance(m, nn.Linear):
                ws.append((m.weight.detach().cpu().numpy().astype(np.float64), m.bias.detach().cpu().numpy().astype(np.float64)))
        ws.append((self.head.weight.detach().cpu().numpy().astype(np.float64), self.head.bias.detach().cpu().numpy().astype(np.float64)))
        return ws, self.log_std.detach().clamp(LOG_STD_MIN, LOG_STD_MAX).cpu().numpy().astype(np.float64)


class Critic(nn.Module):
    def __init__(self, in_dim, hidden=(128, 128, 64)):
        super().__init__()
        dims = [in_dim, *hidden]
        layers = []
        for a, b in zip(dims[:-1], dims[1:]):
            layers += [nn.Linear(a, b), nn.Tanh()]
        layers.append(nn.Linear(dims[-1], 1))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x).squeeze(-1)


def export_json(actor: Actor, path, extra=None, obs_scale=None):
    ws, log_std = actor.numpy_weights()
    d = dict(
        architecture=[actor.body[0].in_features, *HIDDEN, ACT_DIM], activation='tanh', output='linear (clip to +-1 afterwards)',
        obs_scale=(OBS_SCALE if obs_scale is None else np.asarray(obs_scale)).tolist(), obs_clip=5.0,
        layers=[dict(W=W.tolist(), b=b.tolist()) for W, b in ws], log_std=log_std.tolist())
    if extra:
        d.update(extra)
    with open(path, 'w') as f:
        json.dump(d, f)
