# -*- coding: utf-8 -*-
"""기구·액추에이터 (MCP STS3215, PIP/DIP 텐던).

[추정] 값은 가이드 §4 실측(M1~M10) 뒤 여기서만 교체합니다.
"""
# 기구 (AI_Priority_Rules/10 §3.2)
L_PROX, L_MID, L_DIST = 0.040, 0.0158, 0.0155
LINK_W = 0.009            # 링크 두께(캡슐 반경 x2) [추정]
DIP_PIP_RATIO = 0.88      # θ_DIP = 0.88 θ_PIP
TH_MAX = 1.57             # MCP 가동 한계
PIP_MAX = 1.57

# STS3215 (MCP) [가이드 P2-1, 03 §3.1]
STS_KP = 8.0              # Nm/rad [추정]
STS_TAU_MAX = 1.9         # Nm [추정]
STS_VMAX = 4.7            # rad/s [추정]
STS_ARMATURE = 1.0e-3     # kg·m² [추정]
STS_FRICTIONLOSS = 0.01   # Nm [추정]

# 텐던 (PIP/DIP) [가이드 P2-2, 03 §3.2] - 모터 사양 미확정(가이드 M1), TENDON_MOTOR 한 줄로 전환
TENDON_MOTOR = "DS3225"
TENDON_MOTORS = {
    "DS3225": dict(stall_Nm=2.45, clamp=0.30, vmax=5.2),
    "40KG": dict(stall_Nm=3.92, clamp=0.30, vmax=4.4),
}
SPOOL_R = 0.008
R_MCP, R_PIP, R_DIP = 0.0080, 0.0070, 0.0050   # 유효 권선 반경 m/rad [추정]
TENDON_KP = 3000.0        # N/m [추정]
PIP_SPRING = 0.03         # Nm/rad 복귀 스프링 [추정]
TENDON_STROKE = R_PIP * PIP_MAX + R_DIP * PIP_MAX * DIP_PIP_RATIO   # u=1일 때 PIP/DIP 완전 굴곡 길이


def tendon_force_max(motor=None):
    m = TENDON_MOTORS[motor or TENDON_MOTOR]
    return m["stall_Nm"] * m["clamp"] / SPOOL_R
