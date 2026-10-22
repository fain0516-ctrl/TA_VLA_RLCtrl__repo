# -*- coding: utf-8 -*-
"""Motion Mapping: 1조 사람 손 관절각 -> 로봇 손 10 액추에이터 목표 관절각.

입력 [1조 보고서]: 21 개 hand landmark 로 계산한 손가락별 MCP, PIP, DIP 관절각을 ROS2 topic 으로 전달.
"Motion Mapping & Control 파트에서 이를 받아 로봇 손의 목표 관절각으로 변환" 하는 부분이 이 모듈입니다.

로봇 손은 PIP/DIP 가 텐던 하나로 묶여(θ_DIP = κ·θ_PIP) 독립적으로 움직이지 못하므로,
사람의 (PIP, DIP) 두 값을 커플링 직선에 최소제곱으로 투영해 PIP 목표 하나를 만듭니다:
    θ_PIP* = argmin (θ - a)² + (κθ - b)² = (a + κ b) / (1 + κ²)
"""
from dataclasses import dataclass, field

from . import hand_spec as H

# 1조 JointState 의 관절 이름 기본값. 1조와 합의되면 config 에서 바꿉니다.
DEFAULT_HUMAN_NAMES = {
    "thumb": ("thumb_mcp", "thumb_ip"),
    "index": ("index_mcp", "index_pip", "index_dip"),
    "middle": ("middle_mcp", "middle_pip", "middle_dip"),
    "ring": ("ring_mcp", "ring_pip", "ring_dip"),
    "pinky": ("pinky_mcp", "pinky_pip", "pinky_dip"),
}


@dataclass
class MappingParams:
    ratio: float = H.DIP_PIP_RATIO
    # 액추에이터별 q_robot = gain·(q_human - offset). 사람 손 가동 범위 보정용 [추정: 1조 실측 후 교체]
    gain: list = field(default_factory=lambda: [1.0] * H.N_ACTUATORS)
    offset: list = field(default_factory=lambda: [0.0] * H.N_ACTUATORS)
    q_min: float = H.Q_MIN
    q_max: float = H.Q_MAX
    names: dict = field(default_factory=lambda: dict(DEFAULT_HUMAN_NAMES))


def project_pip(pip, dip, ratio):
    if dip is None:
        return pip
    return (pip + ratio * dip) / (1.0 + ratio * ratio)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def map_human_to_robot(human, p: MappingParams, previous=None):
    """human: {관절 이름: rad}. 빠진 관절은 previous(10) 값 또는 0 으로 둡니다.

    반환: (targets[10], missing[이름 목록])
    """
    prev = list(previous) if previous is not None else [0.0] * H.N_ACTUATORS
    raw = list(prev)
    fresh = [False] * H.N_ACTUATORS
    missing = []

    def get(name):
        v = human.get(name)
        if v is None:
            missing.append(name)
        return v

    t_names = p.names["thumb"]
    for k in (0, 1):
        v = get(t_names[k])
        if v is not None:
            raw[k], fresh[k] = v, True

    for f, finger in enumerate(H.FINGERS[1:]):
        mcp_n, pip_n, dip_n = p.names[finger]
        mcp = get(mcp_n)
        pip = get(pip_n)
        dip = human.get(dip_n)          # DIP 는 없어도 PIP 만으로 매핑 가능
        if mcp is not None:
            raw[2 + f], fresh[2 + f] = mcp, True
        if pip is not None:
            raw[6 + f], fresh[6 + f] = project_pip(pip, dip, p.ratio), True

    out = []
    for i, v in enumerate(raw):
        if not fresh[i]:
            out.append(v)           # 이전 출력은 이미 보정·절삭된 값
        else:
            out.append(_clamp(p.gain[i] * (v - p.offset[i]), p.q_min, p.q_max))
    return out, missing
