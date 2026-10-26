# -*- coding: utf-8 -*-
"""정책 행동 -> 관절 목표 적분기 [가이드 P1-2, IndustReal의 policy-level action integrator].

행동 a = [Δq_mcp, Δu_tendon] ∈ [-1, 1]². EMA로 평활한 뒤 스텝당 상한(DQ_MCP, DU_TENDON)을 곱해 누적합니다.
"""
import numpy as np

from .. import config as C


class ActionIntegrator:
    def __init__(self, ema=C.ACT_EMA, dq=C.DQ_MCP, du=C.DU_TENDON):
        self.ema, self.dq, self.du = ema, dq, du
        self.reset(0.0)

    def reset(self, q0, u0=0.0):
        self.mcp_ref, self.u_ref = float(q0), float(u0)
        self.a_f = np.zeros(C.ACT_DIM)

    def step(self, a):
        """반환: (a_f - a_prev) - 행동 변화 패널티 입력."""
        a = np.clip(np.asarray(a, dtype=np.float64).reshape(C.ACT_DIM), -1.0, 1.0)
        a_prev = self.a_f.copy()
        self.a_f = self.ema * a + (1.0 - self.ema) * self.a_f
        self.mcp_ref = float(np.clip(self.mcp_ref + self.a_f[0] * self.dq, 0.0, C.TH_MAX))
        self.u_ref = float(np.clip(self.u_ref + self.a_f[1] * self.du, 0.0, 1.0))
        return self.a_f - a_prev
