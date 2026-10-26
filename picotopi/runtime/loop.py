# -*- coding: utf-8 -*-
"""실행 루프: 25 Hz 정책 + 100 Hz 안전층 (학습 환경과 같은 control 모듈 사용).

    io = SimHardwareIO()                       # 또는 실기 HardwareIO 구현
    loop = ControlLoop(io, NumpyActor("actor.npz"))   # 또는 TactileLatchFSM()
    loop.reset(g=0.5, F_th=2.0)
    for _ in range(75):
        info = loop.step()
    loop.release()                             # 해제 시퀀스 (ADR 005 인터락 경유)

정책 종류
- NumpyActor (또는 obs 벡터를 받아 행동 2개를 내는 callable)
- act_meas(m)를 가진 객체 (TactileLatchFSM) - 측정 합력·목표만 사용
"""
import numpy as np

from .. import config as C
from ..control.action import ActionIntegrator
from ..control.observation import FrameStack, make_frame
from ..control.safety import SafetyLayer, SafetyParams


class ControlLoop:
    def __init__(self, io, policy):
        self.io, self.policy = io, policy
        self.act_int, self.safety, self.stack = ActionIntegrator(), SafetyLayer(), FrameStack()

    def reset(self, g, F_th):
        """g: 목표 파지력 N, F_th: 파손 한계 N (상위 VLA/사용자가 정함)."""
        self.g, self.F_th = float(g), float(F_th)
        s = self.io.read()
        self.act_int.reset(s["q"])
        self.safety.reset(s["q"], SafetyParams(F_th=self.F_th))
        self.has_touched = False
        self.s = s
        self.stack.reset(self._frame(s))
        if hasattr(self.policy, "reset"):
            self.policy.reset()

    def _frame(self, s):
        ai = self.act_int
        return make_frame(s, ai.mcp_ref, ai.u_ref, ai.a_f, self.has_touched)

    def _action(self):
        if hasattr(self.policy, "act_meas"):
            return self.policy.act_meas(dict(F=float(self.s["tac"].sum()), g=self.g))
        return self.policy(self.stack.actor_obs(self.g, self.F_th))

    def _ticks(self):
        F_peak = 0.0
        for _ in range(C.DECIM):
            mcp, ten = self.safety.tick(self.act_int.mcp_ref, self.act_int.u_ref, float(self.s["tac"].sum()),
                                        self.io.tension())
            self.io.write(mcp, ten)
            self.io.wait_tick()
            self.s = self.io.read()
            F_peak = max(F_peak, float(self.s["tac"].sum()))
        return F_peak

    def step(self):
        """정책 1스텝 (= 안전층 DECIM 틱). 반환: 로그용 dict."""
        a = np.asarray(self._action(), dtype=np.float64)
        self.act_int.step(a)
        F_peak = self._ticks()
        if float(self.s["tac"].sum()) >= C.LATCH_F:
            self.has_touched = True
        self.stack.push(self._frame(self.s))
        return dict(a=a, F=float(self.s["tac"].sum()), F_peak_meas=F_peak, q=self.s["q"],
                    mcp_ref=self.act_int.mcp_ref, u_ref=self.act_int.u_ref, yield_e=self.safety.e)

    def release(self, max_steps=50):
        """목표를 0으로 두고 안전층(해제 인터락)이 텐던 -> MCP 순서로 풀도록 진행."""
        self.act_int.mcp_ref, self.act_int.u_ref = 0.0, 0.0
        self.act_int.a_f[:] = 0.0
        for _ in range(max_steps):
            self._ticks()
            if self.safety.mcp_cmd <= 1e-3:
                break
        return self.safety.mcp_cmd
