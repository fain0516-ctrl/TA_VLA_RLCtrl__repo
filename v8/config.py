# -*- coding: utf-8 -*-
"""V8 설정값 (단일 출처).

표기: [논문] 원문 수치, [가이드 §x] docs/research/04_team_guide.md 해당 절, [추정] 실측 전 임시값.
[추정] 값은 04 가이드 §4 실측 항목(M1~M10)이 끝나면 여기서만 바꾸면 됩니다.
"""
import numpy as np

# ------------------------------------------------------------------ 시간 구조 [가이드 P1-2, 03 §2]
SIM_DT = 0.002            # 물리 dt 2 ms (0.5 N 접촉력은 solref에 민감, 03 §2 권장 5 ms 이하)
CTRL_DT = 0.010           # 안전층(슬루·어드미턴스·텐던 인터락) 100 Hz
POLICY_DT = 0.040         # 정책 25 Hz (V7 100 Hz -> 20~25 Hz 권장)
N_SUB = int(round(CTRL_DT / SIM_DT))        # 5
DECIM = int(round(POLICY_DT / CTRL_DT))     # 4
TENDON_ZOH = 0.020        # PCA9685 50 Hz 갱신 가정 [추정, 가이드 M7]
EP_SEC = 3.0
T_EP = int(round(EP_SEC / POLICY_DT))       # 75 정책 스텝

# ------------------------------------------------------------------ 기구 (AI_Priority_Rules/10 §3.2)
L_PROX, L_MID, L_DIST = 0.040, 0.0158, 0.0155
LINK_W = 0.009            # 링크 두께(캡슐 반경 x2) [추정]
DIP_PIP_RATIO = 0.88      # θ_DIP = 0.88 θ_PIP
TH_MAX = 1.57             # MCP 가동 한계
PIP_MAX = 1.57

# ------------------------------------------------------------------ STS3215 (MCP) [가이드 P2-1, 03 §3.1]
STS_KP = 8.0              # Nm/rad 위치 서보 강성 [추정: 사인 스윕 식별 전]
STS_TAU_MAX = 1.9         # Nm (약 19 kg·cm @7.4 V) [추정]
STS_VMAX = 4.7            # rad/s, 7.4 V 무부하 0.22 s/60° [추정: 실측으로 교체]
STS_ARMATURE = 1.0e-3     # kg·m², 기어비² x 로터 관성 [추정: LEAP 1.41e-3, Aero 1.56e-3 참고]
STS_FRICTIONLOSS = 0.01   # Nm [추정: BAM STS3215 모델로 교체 예정]

# ------------------------------------------------------------------ 텐던 (PIP/DIP) [가이드 P2-2, 03 §3.2]
# 텐던 모터 사양 미확정 (가이드 M1: 팀 문서 40KG vs 사용자 설명 DS3225). 여기 한 줄만 바꾸면 됩니다.
TENDON_MOTOR = "DS3225"
TENDON_MOTORS = {           # 스풀 반경 8 mm 가정, 소프트웨어 토크 클램프 30% (10번 문서) [추정]
    "DS3225": dict(stall_Nm=2.45, clamp=0.30, vmax=5.2),   # 약 25 kg·cm
    "40KG": dict(stall_Nm=3.92, clamp=0.30, vmax=4.4),     # 약 40 kg·cm
}
SPOOL_R = 0.008
R_MCP, R_PIP, R_DIP = 0.0080, 0.0070, 0.0050   # 유효 권선 반경 m/rad [추정: CAD 추출 전, Aero 12.5/7.3/9.0 mm 참고]
TENDON_KP = 3000.0        # N/m, 서보 위치 루프를 텐던 길이 강성으로 환산 [추정]
PIP_SPRING = 0.03         # Nm/rad 복귀 스프링 [추정]
TENDON_STROKE = R_PIP * PIP_MAX + R_DIP * PIP_MAX * DIP_PIP_RATIO   # u=1일 때 PIP/DIP 완전 굴곡 길이


def tendon_force_max(motor=None):
    m = TENDON_MOTORS[motor or TENDON_MOTOR]
    return m["stall_Nm"] * m["clamp"] / SPOOL_R


# ------------------------------------------------------------------ 촉각 센서 [가이드 P2-6, M2]
TAC_PADS = ("prox", "mid", "dist")
TAC_THRESH = 0.10         # N, FSR 작동 하한 [추정: M2 실측으로 교체]
TAC_LEVELS = (0.15, 0.5, 1.5)   # 양자화 경계 N -> 0..3 단계 (래치 0.15 N, 목표 0.5 N 영역 구분)
TAC_DROP_P = 0.10         # 패드별 에피소드 드롭 확률 [논문 Yin 2023]
OBS_CONT_FORCE = True     # 연속 힘 관측 사용 여부 (M2 전에는 노이즈·스케일 DR로만 사용)

# ------------------------------------------------------------------ 안전층 [가이드 P1-3]
ADM_M, ADM_D, ADM_K = 0.02, 1.5, 5.0     # 11번 문서 값 (가상 질량·감쇠·강성)
ADM_GAIN = 0.4            # 초과력(N) -> 양보 각 입력 스케일 [추정]
SAFETY_FRAC = 0.9         # 초과력 기준 = 0.9·F_th 에서 양보 시작
RELEASE_T = 2.0           # N, MCP 신전 전 텐던 장력이 이 값 이하여야 함 (ADR 005 해제 시퀀스)

# ------------------------------------------------------------------ 정책 행동 [가이드 P1-2]
DQ_MCP = STS_VMAX * POLICY_DT            # 정책 스텝당 MCP 목표 변화 상한 (rad)
DU_TENDON = 0.25                         # 정책 스텝당 텐던 명령(0~1) 변화 상한
ACT_EMA = 0.6                            # 행동 EMA (AnyRotate 0.5, Touch 0.8 사이)
ACT_DIM = 2                              # [Δq_mcp, Δu_tendon]

# ------------------------------------------------------------------ 과제 [가이드 P1-5]
F_REF = 2.0
F_TH_RANGE = (1.0, 3.0)                  # 파손 한계, 에피소드마다 랜덤화 + Actor 관측 (FORGE)
G_LO, G_HI_FRAC = 0.2, 0.8               # 목표 힘 g ~ U(0.2, 0.8·F_th)
R_FULL = (0.015, 0.040)                  # 물체 반경 최종 범위 (01_PROJECT_GOALS)
R_START = (0.026, 0.034)                 # 커리큘럼 시작 범위
OBJ_GAP = 0.004                          # 손바닥-물체 간격

# ------------------------------------------------------------------ 도메인 랜덤화 [가이드 P2-4, 03 §4.3]
# (nominal_lo, nominal_hi, full_lo, full_hi): ADR이 nominal -> full 사이를 성능 기반으로 확장/축소
DR = dict(
    kp=(0.95, 1.05, 0.8, 1.2),           # STS kp 배율
    tau=(0.95, 1.0, 0.85, 1.0),          # STS 최대 토크 배율 (전압·온도)
    vmax=(0.95, 1.05, 0.85, 1.15),       # STS 최대 속도 배율
    arm=(0.95, 1.05, 0.9, 1.1),          # armature 배율
    zero=(-0.01, 0.01, -0.05, 0.05),     # MCP 엔코더 영점 오프셋 rad
    backlash=(0.0, 0.005, 0.0, 0.026),   # rad (0~1.5°)
    eta=(0.60, 0.70, 0.45, 0.80),        # 텐던 전달효율 (Capstan)
    r_scale=(0.95, 1.05, 0.8, 1.2),      # 권선 반경 배율
    spring=(0.9, 1.1, 0.7, 1.3),         # 복귀 스프링 배율
    slack=(0.0, 0.0005, 0.0, 0.002),     # 텐던 슬랙 m
    mu=(0.8, 1.2, 0.3, 2.0),             # 손끝 마찰
    solref=(0.9, 1.1, 0.5, 2.0),         # 접촉 timeconst 배율
    mass=(0.04, 0.06, 0.02, 0.20),       # 물체 질량 kg
    oppose=(0.9, 1.1, 0.6, 1.5),         # 반대편(엄지·다른 손가락) 지지 강성 배율
    act_delay=(0.0, 0.0, 0.0, 1.5),      # 행동 지연 (안전층 스텝 단위)
    obs_delay=(0.0, 0.01, 0.0, 0.05),    # 관측 지연 s
    tac_scale=(0.9, 1.1, 0.7, 1.3),      # 촉각 스케일
    tac_noise=(0.01, 0.02, 0.01, 0.08),  # 촉각 노이즈 N
    q_noise=(0.002, 0.005, 0.002, 0.01), # 엔코더 스텝 노이즈 rad
)
DR_KEYS = list(DR)
LINEAR_KEYS = {"zero", "backlash", "slack", "act_delay", "obs_delay"}


def dr_bounds(key, phi_lo=0.0, phi_hi=0.0):
    nlo, nhi, flo, fhi = DR[key]
    return nlo + phi_lo * (flo - nlo), nhi + phi_hi * (fhi - nhi)


def dr_sample(key, rng, phi_lo=0.0, phi_hi=0.0, boundary=None):
    """boundary: None | 0(하한 고정) | 1(상한 고정) - ADR 경계 평가용."""
    lo, hi = dr_bounds(key, phi_lo, phi_hi)
    if boundary is not None:
        return float(lo if boundary == 0 else hi)
    if key in LINEAR_KEYS or lo <= 0.0:
        return float(rng.uniform(lo, hi))
    return float(np.exp(rng.uniform(np.log(lo), np.log(hi))))
