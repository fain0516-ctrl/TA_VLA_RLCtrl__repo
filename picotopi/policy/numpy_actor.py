# -*- coding: utf-8 -*-
"""NumPy Actor 추론 - 학습 워커와 실기(Pi) 런타임이 같은 함수를 씁니다.

Actor 구조 (train.nets.Actor와 같은 순서): fc1 -> ReLU -> fc2 -> ReLU -> (mu, log_std), tanh 스쿼시.
Pi에는 torch를 깔지 않도록 .pth를 .npz로 내보내 씁니다:
    python -m picotopi.policy.numpy_actor runs/seed0/final_actor_v8.pth actor.npz
"""
import sys

import numpy as np

PARAM_NAMES = ("fc1.weight", "fc1.bias", "fc2.weight", "fc2.bias", "mu.weight", "mu.bias", "ls.weight", "ls.bias")


def np_policy(p, x, deterministic=False, rng=np.random):
    h = np.maximum(p[0] @ x + p[1], 0.0)
    h = np.maximum(p[2] @ h + p[3], 0.0)
    mu = p[4] @ h + p[5]
    if deterministic:
        return np.tanh(mu)
    ls = np.clip(p[6] @ h + p[7], -5.0, 2.0)
    return np.tanh(mu + np.exp(ls) * rng.standard_normal(mu.shape[0]))


def _load_params(path):
    if str(path).endswith(".npz"):
        z = np.load(path)
        return [z[n] for n in PARAM_NAMES]
    import torch
    sd = torch.load(path, map_location="cpu")
    sd = sd.get("actor", sd)
    return [sd[n].detach().cpu().numpy() for n in PARAM_NAMES]


class NumpyActor:
    """policy 인터페이스(reset/act) + 측정 기반 런타임용 __call__."""

    def __init__(self, path_or_params):
        self.p = _load_params(path_or_params) if isinstance(path_or_params, (str, bytes)) or hasattr(
            path_or_params, "__fspath__") else list(path_or_params)
        self.obs_dim = self.p[0].shape[1]

    def __call__(self, obs_vec, deterministic=True, rng=np.random):
        return np_policy(self.p, obs_vec, deterministic, rng)

    def reset(self, env=None):
        pass

    def act(self, env, obs):
        return self(obs["obs"])


def export_npz(src, dst):
    np.savez(dst, **dict(zip(PARAM_NAMES, _load_params(str(src)))))
    return dst


if __name__ == "__main__":
    export_npz(sys.argv[1], sys.argv[2])
    print(f"saved {sys.argv[2]}")
