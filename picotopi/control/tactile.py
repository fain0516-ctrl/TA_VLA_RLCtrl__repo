# -*- coding: utf-8 -*-
"""촉각 처리 [가이드 P2-6]. 노이즈·스케일·드롭 같은 센서 오차 모델은 sim.env에 있고, 여기에는
실기에서도 똑같이 적용하는 처리(작동 임계, 단계 양자화)만 둡니다."""
import numpy as np

from .. import config as C


def tactile_threshold(tac):
    """FSR 작동 하한 미만은 0 (in-place, 배열 반환)."""
    tac[tac < C.TAC_THRESH] = 0.0
    return tac


def force_level(F_sum):
    """합력(N) -> 0..len(TAC_LEVELS) 단계."""
    return float(np.searchsorted(C.TAC_LEVELS, F_sum, side="right"))
