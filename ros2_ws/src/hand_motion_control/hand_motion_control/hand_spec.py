# -*- coding: utf-8 -*-
"""3조 로봇 손 사양 (단일 출처).

근거 표기
- [3조 보고서] 3조 중간보고서 (2026-09-26)
- [기구학 문서] robotic_hand_ws/kinematics_dynamics_math_model.md, dh_parameters_report.md
- [인터페이스 문서] robotic_hand_ws/ROS2_PIPELINE_ARCHITECTURE.md, SYSTEM_INTERFACE_CONTRACT.md
- [2조 보고서] 2조 와이어 구동 로봇 손 실험 중간보고서 (2026-10-01): 손가락·손바닥 CAD 제원 (기구 제작 담당)
- [추정] 실측 전 임시값. config/hand_params.yaml 에서 덮어씁니다.

구동 구조 [2조 보고서 3·4절, 사용자 결정 2026-11-03]
- 다섯 손가락 모두 첫 번째 관절(뿌리)은 손바닥 안 모터로 직접 구동 (손바닥 내 모터 5개, Feetech STS3215-C018, 시리얼 버스)
- 나머지 관절은 와이어 텐던으로 구동: 서보모터 Reel(horn)에 Tendon 연결 (DS3225 25kg 디지털 서보, 2팀 구매분 5개 [3조 보고서])
  검지~새끼는 텐던 하나가 PIP·DIP 를 함께 굽힘 (θ_DIP = κ·θ_PIP), 엄지는 텐던이 둘째 관절(0-1) 하나를 굽힘
=> 액추에이터 10개 (STS3215 5 + DS3225 5), 관절 14개 (엄지 2 + 4지 x 3)
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
#   0~4 : 엄지~새끼 뿌리 관절 i-0 (STS3215)          -> 손가락 f 는 슬롯 f
#   5~9 : 엄지~새끼 텐던 (DS3225, 엄지 0-1 / 4지 PIP 기준 각) -> 손가락 f 는 슬롯 5+f
ACTUATOR_NAMES = (
    "thumb_mcp", "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp",
    "thumb_tendon", "index_tendon", "middle_tendon", "ring_tendon", "pinky_tendon",
)
N_ACTUATORS = len(ACTUATOR_NAMES)  # 10
N_STS = 5
N_TENDON = 5
STS_SLOTS = tuple(range(0, N_STS))
TENDON_SLOTS = tuple(range(N_STS, N_ACTUATORS))

# 손가락 마디 길이 m [2조 보고서 2. 손가락 주요 제원] (첫째 마디, 둘째 마디, 끝마디)
# 기구는 2조가 만들므로 2조 CAD 값을 씁니다. 표에 단위 표기가 없어 같은 보고서의 손바닥 표(mm)와 같은 mm 로 읽었습니다.
# 마디 부품 길이이므로, 관절 축 사이 거리와 다르면 CAD 실측값으로 바꿔야 합니다 [확인 필요].
# (이전 값: 기구학 문서 1.2 의 0.040 / 0.0158 / 0.0155, 엄지 0.045 / 0.035)
FINGER_LINKS = {
    "index":  (0.0698, 0.0384, 0.0342),   # 기준 총길이 142.4 mm
    "middle": (0.0768, 0.0464, 0.0368),   # 160.0 mm
    "ring":   (0.0722, 0.0445, 0.0369),   # 153.6 mm
    "pinky":  (0.0559, 0.0316, 0.0340),   # 121.6 mm (보고서 표기, 세 마디 합은 121.5)
}
THUMB_LINKS = (0.0553, 0.0471)            # 엄지 2 마디, 102.4 mm

# 손바닥 [2조 보고서 3. 손바닥 주요 제원]
PALM_SIZE = (0.2263, 0.1564, 0.0600)      # 폭 x 길이 x 두께 m (약)
# 손바닥 평면상 손가락 배치 각도 rad: 엄지 50°, 약지·소지 각 8°. 회전 방향(부호)은 보고서에 없음 [확인 필요]
FINGER_YAW = {
    "thumb": math.radians(50.0),
    "index": 0.0,
    "middle": 0.0,
    "ring": math.radians(8.0),
    "pinky": math.radians(8.0),
}

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
    q[0], q[1] = act[STS_SLOTS[0]], act[TENDON_SLOTS[0]]
    for f in range(1, 5):
        mcp = act[STS_SLOTS[f]]
        pip = act[TENDON_SLOTS[f]]
        q[joint_index(f, 0)] = mcp
        q[joint_index(f, 1)] = pip
        q[joint_index(f, 2)] = ratio * pip
    return q
