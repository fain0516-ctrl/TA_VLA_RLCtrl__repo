# -*- coding: utf-8 -*-
"""Actor 관측 [가이드 P1-2, P2-6].

프레임(13) = [q/TH_MAX, mcp_ref/TH_MAX, 위치오차/0.3, u_ref, Load, 이진 촉각 x3, 단계/3, 연속 힘/F_REF, a_f x2, has_touched]
Actor 관측(41) = 최근 H=3 프레임 + [g/F_REF, F_th/F_REF]
측정치 s = dict(tac=(3,) N, q=rad, load=정규화 부하) 만 사용하므로 실기에서 그대로 만들 수 있습니다.
"""
from collections import deque

import numpy as np

from .. import config as C
from .tactile import force_level

FRAME = 13
H = 3
OBS_DIM = FRAME * H + 2
G_IDX = OBS_DIM - 2          # HER 재라벨 시 교체하는 g 위치


def make_frame(s, mcp_ref, u_ref, a_f, has_touched):
    tac = s["tac"]
    Fm = float(tac.sum())
    return np.array([
        s["q"] / C.TH_MAX, mcp_ref / C.TH_MAX, np.clip((mcp_ref - s["q"]) / 0.3, -1, 1), u_ref,
        s["load"], *(tac > 0).astype(np.float64), force_level(Fm) / len(C.TAC_LEVELS),
        min(Fm / C.F_REF, 2.0) if C.OBS_CONT_FORCE else 0.0, *a_f, float(has_touched)],
        dtype=np.float32)


class FrameStack:
    def __init__(self, h=H):
        self.frames = deque(maxlen=h)

    def reset(self, frame):
        self.frames.extend([frame] * self.frames.maxlen)

    def push(self, frame):
        self.frames.append(frame)

    def actor_obs(self, g_N, F_th_N):
        return np.concatenate([*self.frames, [g_N / C.F_REF, F_th_N / C.F_REF]]).astype(np.float32)
