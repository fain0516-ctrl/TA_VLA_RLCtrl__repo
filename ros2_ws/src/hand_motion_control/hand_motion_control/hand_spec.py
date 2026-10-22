# -*- coding: utf-8 -*-
"""3조 로봇 손 사양 (단일 출처).

근거 표기
- [3조 보고서] 3조 중간보고서 (2026-09-26)
- [기구학 문서] robotic_hand_ws/kinematics_dynamics_math_model.md, dh_parameters_report.md
- [인터페이스 문서] robotic_hand_ws/ROS2_PIPELINE_ARCHITECTURE.md, SYSTEM_INTERFACE_CONTRACT.md
- [추정] 실측 전 임시값. config/hand_params.yaml 에서 덮어씁니다.

구동 구조 [3조 보고서]
- 검지·중지·약지·새끼 MCP: 모터 축에 직접 강체 고정하는 직접 구동 (Feetech STS3215-C018 12V 30kg, 시리얼 버스)
- PIP/DIP: 원위부에서 서보모터 Reel(horn)에 Tendon 연결 (DS3225 25kg 디지털 서보, 2팀 구매분)
- 엄지: STS3215 2개 직접 구동 [기구학 문서]
=> 액추에이터 10개 (STS3215 6 + DS3225 4), 관절 14개 (엄지 2 + 4지 x 3)
"""
import math

FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# 14 관절 이름: [기구학 문서] 1.1 의 `i-j` 명명 규칙을 그대로 사용 (i: 0=엄지 .. 4=새끼, j: 0=MCP, 1=PIP, 2=DIP)
JOINT_NAMES = (
    ["joint_0_0", "joint_0_1"]
    + [f"joint_{i}_{j}" for i in range(1, 5) for j in range(3)]
)
N_JOINTS = len(JOINT_NAMES)  # 14

# 10 액추에이터 순서. /control/target_joint_positions 와 /control/actuator_commands 의 배열 순서이기도 합니다.
#   0,1 : 엄지 0-0, 0-1 (STS3215)
#   2~5 : 검지~새끼 MCP i-0 (STS3215)
#   6~9 : 검지~새끼 텐던 i-1 (DS3225, PIP 기준 각)
ACTUATOR_NAMES = (
    "thumb_0_0", "thumb_0_1",
    "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp",
    "index_tendon", "middle_tendon", "ring_tendon", "pinky_tendon",
)
N_ACTUATORS = len(ACTUATOR_NAMES)  # 10
STS_SLOTS = tuple(range(0, 6))
TENDON_SLOTS = tuple(range(6, 10))

# 링크 길이 m [기구학 문서 1.2]
L_PROX, L_MID, L_DIST = 0.040, 0.0158, 0.0155
L_T1, L_T2 = 0.045, 0.035

# 텐던 커플링 θ_DIP = κ·θ_PIP [기구학 문서 1.1, DH 문서 2.1]
# (주의) 레포 v8/config.py 는 0.88 을 씁니다. 2팀 실측 후 한 값으로 맞춰야 합니다.
DIP_PIP_RATIO = 0.8

# 관절 가동 범위 0 ≤ θ ≤ π/2 [기구학 문서 1.1]
Q_MIN = 0.0
Q_MAX = math.pi / 2


def joint_index(finger, j):
    """finger 0..4, j 0..2 -> 14 배열 인덱스."""
    if finger == 0:
        if j not in (0, 1):
            raise ValueError("엄지는 0-0, 0-1 두 관절입니다")
        return j
    if not 1 <= finger <= 4 or j not in (0, 1, 2):
        raise ValueError("잘못된 관절")
    return 2 + (finger - 1) * 3 + j


def actuators_to_joints(act, ratio=DIP_PIP_RATIO):
    """10 액추에이터 관절각(rad) -> 14 관절각. DIP 는 커플링으로 계산."""
    if len(act) != N_ACTUATORS:
        raise ValueError(f"{N_ACTUATORS}개가 필요합니다 ({len(act)}개 수신)")
    q = [0.0] * N_JOINTS
    q[0], q[1] = act[0], act[1]
    for f in range(1, 5):
        mcp = act[1 + f]
        pip = act[5 + f]
        q[joint_index(f, 0)] = mcp
        q[joint_index(f, 1)] = pip
        q[joint_index(f, 2)] = ratio * pip
    return q
