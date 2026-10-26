# -*- coding: utf-8 -*-
"""ADR(경계 평가 기반 DR 범위 자동 확장) + 반경 커리큘럼 [가이드 P1-5, P2-4].

ADR: 워커가 p_boundary 확률로 (키, 하한|상한)을 경계값에 고정한 에피소드를 돌리고, 그 측면의 유지 비율 평균이
     adr_hi 초과면 확장도 phi += adr_step, adr_lo 미만이면 -= adr_step (adr_n 에피소드마다 판정).
커리큘럼: 경계 평가가 아닌 에피소드의 유지 비율 평균(curr_n개)이 curr_thr 이상이면 curr += curr_step.
"""
import numpy as np

from .. import config as C

ADR_SIDES = C.adr_sides()


class ADRController:
    def __init__(self, n=20, step=0.05, hi=0.30, lo=0.10):
        self.n, self.step, self.hi, self.lo = n, step, hi, lo
        self.phi = np.zeros((len(C.DR_KEYS), 2))
        self.buf = [[] for _ in ADR_SIDES]

    def record(self, side_idx, hold):
        """반환: phi가 바뀌었으면 True."""
        ab = self.buf[side_idx]
        ab.append(hold)
        if len(ab) < self.n:
            return False
        mh, (j, sd) = float(np.mean(ab)), ADR_SIDES[side_idx]
        ab.clear()
        old = self.phi[j, sd]
        if mh > self.hi:
            self.phi[j, sd] = min(1.0, old + self.step)
        elif mh < self.lo:
            self.phi[j, sd] = max(0.0, old - self.step)
        return self.phi[j, sd] != old


class RadiusCurriculum:
    def __init__(self, thr=0.30, step=0.1, n=100):
        from collections import deque
        self.thr, self.step = thr, step
        self.value = 0.0
        self.buf = deque(maxlen=n)

    def add(self, hold):
        self.buf.append(hold)

    def advance(self):
        """에피소드 묶음마다 1회 호출. 반환: 단계가 올라갔으면 True."""
        if len(self.buf) >= self.buf.maxlen and np.mean(self.buf) >= self.thr and self.value < 1.0:
            self.value = min(1.0, self.value + self.step)
            self.buf.clear()
            return True
        return False
