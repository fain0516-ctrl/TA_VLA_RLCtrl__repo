# -*- coding: utf-8 -*-
"""PICOtoPI 모듈 테스트: python -m pytest tests/test_picotopi.py -q

핵심 검증: 모듈로 재조립한 환경이 V8 원본과 같은 시드·행동에서 비트 단위로 같은 궤적을 냄.
"""
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from picotopi import config as C
from picotopi.control import TactileLatchFSM, Still
from picotopi.reward import GraspReward
from picotopi.sim import FingerGraspEnv, OBS_DIM


def test_config_matches_v8():
    from v8 import config as V
    for k in dir(V):
        if k.isupper():
            assert getattr(C, k) == getattr(V, k), k


@pytest.mark.parametrize("seed,full", [(0, False), (1, False), (5, True)])
def test_env_identical_to_v8(seed, full):
    from v8.env import FingerGraspEnvV8
    e1, e2 = FingerGraspEnvV8(seed=seed), FingerGraspEnv(seed=seed)
    if full:
        e1.adr[:] = e2.adr[:] = 1.0
        e1.curr = e2.curr = 1.0
    a_rng = np.random.default_rng(100 + seed)
    o1, o2 = e1.reset(), e2.reset()
    for k in o1:
        assert np.array_equal(o1[k], o2[k]), k
    while True:
        a = a_rng.uniform(-1, 1, 2)
        (o1, d1, i1), (o2, d2, i2) = e1.step(a), e2.step(a)
        for k in o1:
            assert np.array_equal(o1[k], o2[k]), k
        for k in i1:
            assert np.array_equal(i1[k], i2[k]), k
        assert d1 == d2
        if d1:
            break


def test_reward_alias_and_terms():
    from v8.reward import RewardV8
    r, v = GraspReward(), RewardV8()
    args = (0.003, np.array([0.1, -0.2]), 2.3, 2.0, True)
    assert r.nongoal_terms(*args) == v.nongoal_terms(*args)


def test_fsm_baseline_tracks_goal():
    env = FingerGraspEnv(seed=2)
    pol = TactileLatchFSM()
    o = env.reset(params=dict(radius=0.026, g=0.5, F_th=2.0, drop=np.zeros(3)))
    pol.reset()
    late = []
    for t in range(C.T_EP):
        o, done, info = env.step(pol.act(env, o))
        if t >= C.T_EP - 20 and info["contact"]:
            late.append(info["F_bar"])
        if done:
            break
    assert late and abs(np.mean(late) - 0.5) < 0.3


def test_runtime_loop_with_sim_io():
    from picotopi.runtime import ControlLoop, SimHardwareIO
    io = SimHardwareIO()
    loop = ControlLoop(io, TactileLatchFSM())
    loop.reset(g=0.5, F_th=2.0)
    F = [loop.step()["F"] for _ in range(C.T_EP)]
    assert abs(np.mean(F[-20:]) - 0.5) < 0.3
    assert loop.release() <= 1e-3


def test_numpy_actor_matches_torch_and_npz(tmp_path):
    torch = pytest.importorskip("torch")
    from picotopi.policy import NumpyActor, export_npz
    from picotopi.train.nets import Actor
    net = Actor(OBS_DIM, 32)
    torch.nn.init.normal_(net.mu.weight)
    pth = tmp_path / "a.pth"
    torch.save(net.state_dict(), pth)
    npz = export_npz(str(pth), str(tmp_path / "a.npz"))
    x = np.random.default_rng(0).standard_normal(OBS_DIM).astype(np.float32)
    with torch.no_grad():
        h = torch.relu(net.fc2(torch.relu(net.fc1(torch.as_tensor(x)))))
        ref = torch.tanh(net.mu(h)).numpy()
    for src in (str(pth), npz):
        assert np.allclose(NumpyActor(src)(x), ref, atol=1e-5)


def test_replay_her_and_learner_update():
    torch = pytest.importorskip("torch")
    from picotopi.control.observation import G_IDX
    from picotopi.policy import np_policy
    from picotopi.train.learner import Learner
    from picotopi.train.loop import CFG
    from picotopi.train.replay import EpisodeReplay
    lr = Learner(dict(CFG, hidden=32), torch.device("cpu"))
    p = [t.detach().numpy() for t in lr.actor.parameters()]
    env, rng, eps = FingerGraspEnv(seed=0), np.random.default_rng(0), []
    for _ in range(3):
        o = env.reset()
        ep = dict(obs=[o["obs"]], ag=[o["ag"]], pd=[o["pd"]], act=[], r_np=[], term=[], g=np.array([o["g"]]),
                  ps=o["ps"])
        while True:
            a = np_policy(p, o["obs"], rng=rng)
            o, done, info = env.step(a)
            for k in ("obs", "ag", "pd"):
                ep[k].append(o[k])
            ep["act"].append(a); ep["r_np"].append(info["r_np"]); ep["term"].append(float(info["term"]))
            if done:
                break
        eps.append({k: np.asarray(v, dtype=np.float32) for k, v in ep.items()})
    buf = EpisodeReplay(8, C.T_EP, torch.device("cpu"))
    assert buf.add(eps) == sum(len(e["act"]) for e in eps)
    B = buf.sample(64, GraspReward(), 1.0)
    assert B["obs"].shape == (64, OBS_DIM) and torch.isfinite(B["r"]).all()
    assert torch.equal(B["obs"][:, G_IDX], B["obs2"][:, G_IDX])
    lr.update(B)
    assert all(np.isfinite(lr.stat))


def test_adr_controller_expands_and_shrinks():
    from picotopi.train.schedulers import ADRController, ADR_SIDES, RadiusCurriculum
    adr = ADRController(n=2)
    j, s = ADR_SIDES[0]
    adr.record(0, 0.5); assert adr.record(0, 0.5) and adr.phi[j, s] == pytest.approx(0.05)
    adr.record(0, 0.0); assert adr.record(0, 0.0) and adr.phi[j, s] == 0.0
    cur = RadiusCurriculum(n=2)
    cur.add(0.5); cur.add(0.5)
    assert cur.advance() and cur.value == pytest.approx(0.1)
