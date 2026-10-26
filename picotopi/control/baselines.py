# -*- coding: utf-8 -*-
"""비교 기준 정책 [가이드 P0-2, P0-4]. 측정치만 사용 (env.measured() 또는 runtime 측정 dict).

- still : 가만히. 수익이 random보다 높으면 패널티 과대
- random: 균등 무작위
- fsm   : 스위프 접근 -> 촉각 래치 -> 측정력 P 제어 (09번 문서 구조)
"""
import numpy as np

from .. import config as C


class Still:
    def reset(self, env=None):
        pass

    def act(self, env, obs):
        return np.zeros(C.ACT_DIM)


class Random:
    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)

    def reset(self, env=None):
        pass

    def act(self, env, obs):
        return self.rng.uniform(-1, 1, C.ACT_DIM)


class TactileLatchFSM:
    def __init__(self, sweep=0.3, latch_F=C.LATCH_F, kp=0.3):
        self.sweep, self.latch_F, self.kp = sweep, latch_F, kp

    def reset(self, env=None):
        self.latched = False

    def act_meas(self, m):
        """m: dict(F=측정 합력 N, g=목표 힘 N)."""
        if not self.latched and m["F"] >= self.latch_F:
            self.latched = True
        if not self.latched:
            return np.array([self.sweep, self.sweep])
        u = float(np.clip(self.kp * (m["g"] - m["F"]) / C.F_REF, -1, 1))
        return np.array([u, u])

    def act(self, env, obs):
        return self.act_meas(env.measured())


BASELINES = dict(still=Still, random=Random, fsm=TactileLatchFSM)
