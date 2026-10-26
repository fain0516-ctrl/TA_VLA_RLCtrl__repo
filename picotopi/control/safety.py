# -*- coding: utf-8 -*-
"""100 Hz 안전층 [가이드 P1-3, DextrAH-G의 '정책 아래 안전 제어층' 구조].

정책(25 Hz)이 낸 목표 (mcp_ref, u_ref)를 받아 서보 명령으로 바꾸기 전에 매 틱 다음을 적용합니다.
  (1) 초과력 어드미턴스 양보: 측정력이 SAFETY_FRAC·F_th 를 넘는 만큼만 MCP·텐던 목표를 뒤로 뺌
  (2) 해제 인터락 (ADR 005): 텐던 장력이 RELEASE_T 를 넘으면 MCP 신전 보류
  (3) 슬루 제한: STS_VMAX x 배율
  (4) 백래시 데드밴드 (시뮬 DR용. 실기에서는 backlash=0)
  (5) 텐던 PWM ZOH + MCP 연성 피드포워드 ΔL = r_mcp·q_mcp + 슬랙
시뮬과 실기가 같은 클래스를 씁니다. 실기에서는 tension 대신 추정값(없으면 0)을 넘깁니다.
"""
from dataclasses import dataclass

import numpy as np

from .. import config as C


@dataclass
class SafetyParams:
    F_th: float = 2.0
    vmax: float = 1.0          # STS 속도 배율
    backlash: float = 0.0      # rad
    r_scale: float = 1.0       # 권선 반경 배율
    slack: float = 0.0         # m

    @classmethod
    def from_episode(cls, p):
        return cls(F_th=p["F_th"], vmax=p["vmax"], backlash=p["backlash"], r_scale=p["r_scale"], slack=p["slack"])


class SafetyLayer:
    def __init__(self, params=None):
        self.prm = params or SafetyParams()
        self.reset(0.0)

    def reset(self, q0, params=None):
        if params is not None:
            self.prm = params
        self.mcp_cmd = self.mcp_bl = float(q0)    # 슬루 후 명령, 백래시 적용 서보 목표
        self.tendon_ctrl, self.zoh_t = 0.0, 0.0
        self.e, self.ed = 0.0, 0.0                # 어드미턴스 양보 각과 속도

    def tick(self, mcp_ref, u_ref, F_meas, tension):
        """반환: (MCP 서보 목표 rad, 텐던 길이 명령 m - 음수가 당김)."""
        p = self.prm
        f_in = C.ADM_GAIN * max(F_meas - C.SAFETY_FRAC * p.F_th, 0.0)
        edd = (f_in - C.ADM_D * self.ed - C.ADM_K * self.e) / C.ADM_M
        self.ed += edd * C.CTRL_DT
        self.e = max(self.e + self.ed * C.CTRL_DT, 0.0)
        if self.e == 0.0:
            self.ed = max(self.ed, 0.0)
        tgt = float(np.clip(mcp_ref - self.e, 0.0, C.TH_MAX))
        u = float(np.clip(u_ref - self.e / C.PIP_MAX, 0.0, 1.0))
        if tgt < self.mcp_cmd and tension > C.RELEASE_T:
            tgt = self.mcp_cmd
        dmax = C.STS_VMAX * p.vmax * C.CTRL_DT
        self.mcp_cmd += float(np.clip(tgt - self.mcp_cmd, -dmax, dmax))
        bl = p.backlash / 2
        if self.mcp_cmd > self.mcp_bl + bl:
            self.mcp_bl = self.mcp_cmd - bl
        elif self.mcp_cmd < self.mcp_bl - bl:
            self.mcp_bl = self.mcp_cmd + bl
        self.zoh_t += C.CTRL_DT
        if self.zoh_t >= C.TENDON_ZOH - 1e-9:
            self.zoh_t = 0.0
            rs = p.r_scale
            self.tendon_ctrl = -(C.R_MCP * rs * self.mcp_cmd + u * C.TENDON_STROKE * rs) + p.slack
        return self.mcp_bl, self.tendon_ctrl
