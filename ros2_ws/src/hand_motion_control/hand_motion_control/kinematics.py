# -*- coding: utf-8 -*-
"""순기구학과 텐던 길이 모델.

- 손끝 위치: [기구학 문서] 2.3 폐형태 수식 (MCP 원점, 굴곡 평면 X-Z)
- 텐던 길이 L(θ): 3조 보고서의 "Pin joint 에서 원하는 굽힘각(θ)일 때 조절해야 할 Tendon 의 길이(L)".
  여기서는 관절마다 유효 권선 반경이 일정한 모델 L = r_MCP·θ_MCP + r_PIP·θ_PIP + r_DIP·θ_DIP 를 씁니다.
  MCP 항은 [AI_Priority_Rules/10] 2.1 Kinematic Decoupler (ΔL = r_mcp·Δq_mcp) 와 같습니다.
  3조가 논문에서 유도한 수식이 확정되면 tendon_length() 만 바꾸면 됩니다.
- 반경 기본값은 레포 v8/config.py 의 [추정] 값(R_MCP 8 mm, R_PIP 7 mm, R_DIP 5 mm, 스풀 8 mm)과 같습니다.
"""
import math
from dataclasses import dataclass

from . import hand_spec as H


@dataclass
class TendonParams:
    r_mcp: float = 0.0080     # m/rad [추정]
    r_pip: float = 0.0070     # m/rad [추정]
    r_dip: float = 0.0050     # m/rad [추정]
    r_pulley: float = 0.0080  # 서보 horn(Reel) 반경 m [추정]
    ratio: float = H.DIP_PIP_RATIO


def finger_tip(q_mcp, q_pip, ratio=H.DIP_PIP_RATIO):
    """검지~새끼 손끝 (x, z) m. [기구학 문서] 2.2~2.3."""
    p0 = q_mcp
    p1 = q_mcp + q_pip
    p2 = q_mcp + (1.0 + ratio) * q_pip
    x = H.L_PROX * math.cos(p0) + H.L_MID * math.cos(p1) + H.L_DIST * math.cos(p2)
    z = H.L_PROX * math.sin(p0) + H.L_MID * math.sin(p1) + H.L_DIST * math.sin(p2)
    return x, z


def thumb_tip_local(q0, q1):
    """엄지 국소 평면 손끝 (x, z) m. [기구학 문서] 5."""
    x = H.L_T1 * math.cos(q0) + H.L_T2 * math.cos(q0 + q1)
    z = H.L_T1 * math.sin(q0) + H.L_T2 * math.sin(q0 + q1)
    return x, z


def tendon_length(q_mcp, q_pip, p: TendonParams):
    """완전히 편 자세 기준 텐던 감김 길이 L (m)."""
    return p.r_mcp * q_mcp + (p.r_pip + p.ratio * p.r_dip) * q_pip


def tendon_servo_angle(q_mcp, q_pip, p: TendonParams):
    """목표 (MCP, PIP) 를 만들기 위한 텐던 서보 horn 회전각 (rad)."""
    return tendon_length(q_mcp, q_pip, p) / p.r_pulley


def pip_from_servo_angle(servo_angle, q_mcp, p: TendonParams):
    """tendon_servo_angle 의 역함수 (시뮬레이션·피드백 추정용)."""
    k = p.r_pip + p.ratio * p.r_dip
    return (servo_angle * p.r_pulley - p.r_mcp * q_mcp) / k
