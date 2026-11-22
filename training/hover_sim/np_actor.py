"""Torch-free actor used by rollout workers, evaluation and the C++ cross-check."""
import json

import numpy as np


class NumpyActor:
    def __init__(self, weights, log_std=None):
        self.w = weights
        self.log_std = log_std

    def mean(self, obs):
        x = np.asarray(obs, dtype=np.float64)
        for W, b in self.w[:-1]:
            x = np.tanh(x @ W.T + b)
        W, b = self.w[-1]
        return x @ W.T + b

    @classmethod
    def from_json(cls, path):
        d = json.loads(open(path).read())
        w = [(np.array(l['W'], dtype=np.float64), np.array(l['b'], dtype=np.float64)) for l in d['layers']]
        return cls(w, np.array(d.get('log_std', [-3.0, -3.0])))
