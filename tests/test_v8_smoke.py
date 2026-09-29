# -*- coding: utf-8 -*-
"""V8 스모크 테스트: python -m pytest tests/test_v8_smoke.py -q"""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from v8 import config as C
from v8.env import FingerGraspEnvV8, OBS_DIM, PS_DIM, PD_DIM
from v8.baselines import TactileLatchFSM, Still
from v8.reward import RewardV8


def _rollout(env, pol, params=None):
    o = env.reset(params=params)
    pol.reset(env)
    infos = []
    while True:
        o, done, info = env.step(pol.act(env, o))
        infos.append(info)
        if done:
            return o, infos


def test_shapes_and_episode_length():
    env = FingerGraspEnvV8(seed=0)
    o, infos = _rollout(env, Still())
    assert o["obs"].shape == (OBS_DIM,) and o["ps"].shape == (PS_DIM,) and o["pd"].shape == (PD_DIM,)
    assert len(infos) == C.T_EP and np.isfinite(o["obs"]).all()


def test_goal_and_force_limit_in_actor_obs():
    env = FingerGraspEnvV8(seed=1)
    o = env.reset()
    assert o["obs"][-2] == pytest.approx(env.p["g"] / C.F_REF)
    assert o["obs"][-1] == pytest.approx(env.p["F_th"] / C.F_REF)


def test_fsm_makes_contact_and_tracks_force():
    env = FingerGraspEnvV8(seed=2)
    _, infos = _rollout(env, TactileLatchFSM(), dict(radius=0.026, g=0.5, F_th=2.0, drop=np.zeros(3)))
    assert any(i["contact"] for i in infos)
    late = [i["F_bar"] for i in infos[-20:] if i["contact"]]
    assert late and abs(np.mean(late) - 0.5) < 0.3


def test_tendon_independent_of_mcp():
    env = FingerGraspEnvV8(seed=3)
    env.reset(params=dict(radius=0.04, drop=np.zeros(3)))
    for _ in range(20):
        env.step(np.array([0.0, 1.0]))      # 텐던만 당김
    assert env.data.qpos[1] > 0.3 and env.data.qpos[0] < 0.15


def test_overforce_is_linear_not_cliff():
    r = RewardV8()
    _, p1 = r.nongoal_terms(0.0, np.zeros(2), 2.2, 2.0, False)
    _, p2 = r.nongoal_terms(0.0, np.zeros(2), 2.4, 2.0, False)
    assert p2["over"] == pytest.approx(2 * p1["over"])
    assert 0 < r.cat_term_prob(p1["excess"]) < r.cat_term_prob(p2["excess"]) <= r.cat_pmax


def test_goal_terms_numpy_torch_agree():
    torch = pytest.importorskip("torch")
    r = RewardV8()
    hist = [0.48, 0.5, 0.52, 0.49, 0.51]
    a, h = r.goal_terms_np(0.51, 1.0, hist, 0.5)
    b, h2 = r.goal_terms_torch(torch.tensor([0.51]), torch.tensor([1.0]), torch.tensor([hist]), torch.tensor([0.5]))
    assert a == pytest.approx(float(b[0]), rel=1e-5) and h == 1.0 and float(h2[0]) == 1.0


def test_penalty_check_passes():
    from v8.evaluate import penalty_check
    ok, out = penalty_check(n_ep=10)
    assert ok, out
