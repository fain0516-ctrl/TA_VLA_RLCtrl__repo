# -*- coding: utf-8 -*-
"""안전층 [가이드 P1-3] · 정책 행동 [가이드 P1-2]."""
from .mechanics import STS_VMAX
from .timing import POLICY_DT

ADM_M, ADM_D, ADM_K = 0.02, 1.5, 5.0     # 가상 질량·감쇠·강성 (11번 문서)
ADM_GAIN = 0.4            # 초과력(N) -> 양보 각 입력 스케일 [추정]
SAFETY_FRAC = 0.9         # 0.9·F_th 에서 양보 시작
RELEASE_T = 2.0           # N, MCP 신전 전 텐던 장력 상한 (ADR 005 해제 시퀀스)

DQ_MCP = STS_VMAX * POLICY_DT            # 정책 스텝당 MCP 목표 변화 상한 (rad)
DU_TENDON = 0.25                         # 정책 스텝당 텐던 명령(0~1) 변화 상한
ACT_EMA = 0.6                            # 행동 EMA
ACT_DIM = 2                              # [Δq_mcp, Δu_tendon]
