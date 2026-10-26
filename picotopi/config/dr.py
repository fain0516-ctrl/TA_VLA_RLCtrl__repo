# -*- coding: utf-8 -*-
"""도메인 랜덤화 표와 샘플러 [가이드 P2-4, 03 §4.3].

각 키: (nominal_lo, nominal_hi, full_lo, full_hi). ADR 확장도 phi(0~1)가 nominal -> full 사이를 보간합니다.
"""
import numpy as np

DR = dict(
    kp=(0.95, 1.05, 0.8, 1.2),           # STS kp 배율
    tau=(0.95, 1.0, 0.85, 1.0),          # STS 최대 토크 배율
    vmax=(0.95, 1.05, 0.85, 1.15),       # STS 최대 속도 배율
    arm=(0.95, 1.05, 0.9, 1.1),          # armature 배율
    zero=(-0.01, 0.01, -0.05, 0.05),     # MCP 엔코더 영점 오프셋 rad
    backlash=(0.0, 0.005, 0.0, 0.026),   # rad
    eta=(0.60, 0.70, 0.45, 0.80),        # 텐던 전달효율
    r_scale=(0.95, 1.05, 0.8, 1.2),      # 권선 반경 배율
    spring=(0.9, 1.1, 0.7, 1.3),         # 복귀 스프링 배율
    slack=(0.0, 0.0005, 0.0, 0.002),     # 텐던 슬랙 m
    mu=(0.8, 1.2, 0.3, 2.0),             # 손끝 마찰
    solref=(0.9, 1.1, 0.5, 2.0),         # 접촉 timeconst 배율
    mass=(0.04, 0.06, 0.02, 0.20),       # 물체 질량 kg
    oppose=(0.9, 1.1, 0.6, 1.5),         # 반대편 지지 강성 배율
    act_delay=(0.0, 0.0, 0.0, 1.5),      # 행동 지연 (안전층 틱)
    obs_delay=(0.0, 0.01, 0.0, 0.05),    # 관측 지연 s
    tac_scale=(0.9, 1.1, 0.7, 1.3),      # 촉각 스케일
    tac_noise=(0.01, 0.02, 0.01, 0.08),  # 촉각 노이즈 N
    q_noise=(0.002, 0.005, 0.002, 0.01), # 엔코더 노이즈 rad
)
DR_KEYS = list(DR)
LINEAR_KEYS = {"zero", "backlash", "slack", "act_delay", "obs_delay"}


def dr_bounds(key, phi_lo=0.0, phi_hi=0.0):
    nlo, nhi, flo, fhi = DR[key]
    return nlo + phi_lo * (flo - nlo), nhi + phi_hi * (fhi - nhi)


def dr_sample(key, rng, phi_lo=0.0, phi_hi=0.0, boundary=None):
    """boundary: None | 0(하한 고정) | 1(상한 고정) - ADR 경계 평가용. 배율 키는 로그 균등."""
    lo, hi = dr_bounds(key, phi_lo, phi_hi)
    if boundary is not None:
        return float(lo if boundary == 0 else hi)
    if key in LINEAR_KEYS or lo <= 0.0:
        return float(rng.uniform(lo, hi))
    return float(np.exp(rng.uniform(np.log(lo), np.log(hi))))


def adr_sides():
    """확장 가능한 (키 인덱스, 0=하한|1=상한) 목록. nominal과 full이 같은 쪽은 제외."""
    return [(j, s) for j, k in enumerate(DR_KEYS) for s in (0, 1) if DR[k][s] != DR[k][s + 2]]
