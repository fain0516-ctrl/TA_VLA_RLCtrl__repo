# -*- coding: utf-8 -*-
"""비교 기준 정책 [가이드 P0-2, P0-4].

- still : 가만히 있는 정책. 이 정책의 수익이 random보다 높으면 패널티가 과대 (V6 jerk -0.5 실패 탐지)
- random: 균등 무작위 행동
- fsm   : 09번 문서의 스위프 접근 -> 촉각 래치(0.15 N) -> 힘 추종. RL이 이 기준선보다 나아야 의미가 있음
모두 실기에서 볼 수 있는 측정치(env.measured())만 사용합니다.
"""
import numpy as np

from . import config as C


class Still:
    def reset(self, env):
        pass

    def act(self, env, obs):
        return np.zeros(C.ACT_DIM)


class Random:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)

    def reset(self, env):
        pass

    def act(self, env, obs):
        return self.rng.uniform(-1, 1, C.ACT_DIM)


class TactileLatchFSM:
    """스위프 접근 -> 래치 -> 측정력 P 제어 (09번 문서 구조를 V8 행동 공간으로 옮긴 것)."""

    def __init__(self, sweep=0.3, latch_F=0.15, kp=0.3):
        self.sweep, self.latch_F, self.kp = sweep, latch_F, kp

    def reset(self, env):
        self.latched = False

    def act(self, env, obs):
        m = env.measured()
        if not self.latched and m["F"] >= self.latch_F:
            self.latched = True
        if not self.latched:
            return np.array([self.sweep, self.sweep])
        e = (m["g"] - m["F"]) / C.F_REF
        u = float(np.clip(self.kp * e, -1, 1))
        return np.array([u, u])


BASELINES = dict(still=Still, random=Random, fsm=TactileLatchFSM)
