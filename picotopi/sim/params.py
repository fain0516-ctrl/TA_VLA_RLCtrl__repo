# -*- coding: utf-8 -*-
"""에피소드 파라미터 샘플링: DR(ADR 확장도 반영) + 반경 커리큘럼 + 과제(F_th, g) + 촉각 패드 드롭.

난수 소비 순서는 V8 env와 같습니다 (같은 시드 -> 같은 에피소드).
"""
import numpy as np

from .. import config as C


def radius_range(curr):
    """curr 0(시작 범위) ~ 1(전체 범위)."""
    lo = C.R_START[0] + curr * (C.R_FULL[0] - C.R_START[0])
    hi = C.R_START[1] + curr * (C.R_FULL[1] - C.R_START[1])
    return lo, hi


def sample_episode_params(rng, adr, curr, boundary=None):
    """adr: (len(DR_KEYS), 2) 확장도, boundary: None | (키 인덱스, 0|1)."""
    p = {}
    for j, k in enumerate(C.DR_KEYS):
        b = boundary[1] if boundary is not None and boundary[0] == j else None
        p[k] = C.dr_sample(k, rng, adr[j, 0], adr[j, 1], b)
    p["radius"] = float(rng.uniform(*radius_range(curr)))
    p["tendon_fmax"] = C.tendon_force_max()
    p["F_th"] = float(rng.uniform(*C.F_TH_RANGE))
    p["g"] = float(rng.uniform(C.G_LO, C.G_HI_FRAC * p["F_th"]))
    p["drop"] = (rng.random(3) < C.TAC_DROP_P).astype(np.float64)
    return p
