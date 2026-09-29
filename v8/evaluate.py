#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V8 평가 프로토콜 [가이드 P0-2, P0-3, P0-4, §3.4].

지표 (반경 구간별):
- no_crush : 에피소드 중 순간 최대 접촉력 F_peak <= F_th 인 비율 (3스텝 지속 규칙 없음)
- rms      : 유지 구간(후반 절반) 접촉 스텝의 |F - g| RMS (N)
- hold     : 유지 판정(m_hold 스텝 연속 허용오차 이내) 스텝 비율
- slip     : 접촉 상실 에지가 한 번이라도 난 비율
- ret      : 에피소드 수익 (--penalty-check 용)

예)
  python -m v8.evaluate --policy fsm                       # FSM 기준선
  python -m v8.evaluate --policy v8/runs/seed0/final_actor_v8.pth --seeds 0 1 2
  python -m v8.evaluate --penalty-check                    # still / random / fsm 수익 비교
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from v8 import config as C                          # noqa: E402
from v8.env import FingerGraspEnvV8, OBS_DIM        # noqa: E402
from v8.baselines import BASELINES                  # noqa: E402

RADII = (0.015, 0.020, 0.026, 0.034, 0.040)


class TorchPolicy:
    def __init__(self, path):
        import torch
        from v8.train_sac_her_v8 import Actor
        sd = torch.load(path, map_location="cpu")
        sd = sd.get("actor", sd)
        self.net = Actor(OBS_DIM, sd["fc1.weight"].shape[0])
        self.net.load_state_dict(sd)
        self.p = [t.detach().numpy() for t in self.net.parameters()]

    def reset(self, env):
        pass

    def act(self, env, obs):
        from v8.train_sac_her_v8 import np_policy
        return np_policy(self.p, obs["obs"], deterministic=True)


def make_policy(name):
    if name in BASELINES:
        return BASELINES[name]()
    return TorchPolicy(name)


def run_episode(env, pol, params):
    o = env.reset(params=params)
    pol.reset(env)
    ret, hold, peak, slip, err = 0.0, 0.0, 0.0, False, []
    while True:
        o, done, info = env.step(pol.act(env, o))
        ret += info["r"]
        hold += info["hold"]
        peak = max(peak, info["F_peak"])
        slip |= info["slip_edge"]
        if env.steps > C.T_EP // 2 and info["contact"]:
            err.append(info["F_bar"] - env.p["g"])
        if done:
            break
    rms = float(np.sqrt(np.mean(np.square(err)))) if err else float("nan")
    return dict(ret=ret, hold=hold / C.T_EP, no_crush=float(peak <= env.p["F_th"]), rms=rms, slip=float(slip),
                peak=peak)


def evaluate(policy, seeds=(0,), n_ep=20, adr_full=False, radii=RADII):
    rows = {}
    for r in radii:
        res = []
        for sd in seeds:
            env = FingerGraspEnvV8(seed=sd)
            if adr_full:
                env.adr[:] = 1.0
            pol = make_policy(policy)
            if hasattr(pol, "rng"):
                pol.rng = np.random.default_rng(sd)
            for _ in range(n_ep):
                res.append(run_episode(env, pol, dict(radius=r)))
        rows[r] = {k: float(np.nanmean([x[k] for x in res])) for k in res[0]}
    return rows


def print_table(name, rows):
    print(f"\n== {name}")
    print(f"{'radius':>7} {'no_crush':>9} {'hold':>6} {'rms[N]':>7} {'slip':>5} {'peak[N]':>8} {'ret':>7}")
    for r, v in rows.items():
        print(f"{r * 100:6.1f}cm {v['no_crush']:9.2f} {v['hold']:6.3f} {v['rms']:7.3f} {v['slip']:5.2f} "
              f"{v['peak']:8.2f} {v['ret']:7.2f}")


def penalty_check(n_ep=30, seed=0):
    """가만히 있는 정책의 수익이 무작위 정책보다 높으면 패널티 과대 (V6 jerk -0.5 실패 사전 탐지)."""
    out = {}
    for name in ("still", "random", "fsm"):
        rows = evaluate(name, seeds=(seed,), n_ep=n_ep // len(RADII) + 1)
        out[name] = float(np.mean([v["ret"] for v in rows.values()]))
    print("\n== penalty check (episode return)")
    for k, v in out.items():
        print(f"{k:>7}: {v:8.2f}")
    ok = out["still"] <= out["random"] and out["fsm"] > out["still"]
    print("PASS" if ok else "FAIL: still >= random 또는 fsm <= still -> 패널티 규모 재점검 (가이드 P0-2)")
    return ok, out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="fsm", help="still | random | fsm | actor .pth 경로")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--adr-full", action="store_true", help="DR 전체 범위에서 평가")
    ap.add_argument("--penalty-check", action="store_true")
    a = ap.parse_args()
    if a.penalty_check:
        sys.exit(0 if penalty_check()[0] else 1)
    print_table(a.policy, evaluate(a.policy, tuple(a.seeds), a.episodes, a.adr_full))
