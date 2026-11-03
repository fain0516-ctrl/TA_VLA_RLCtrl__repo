# -*- coding: utf-8 -*-
"""하위 제어기 핵심 로직 (ROS 의존 없음, 100 Hz tick).

3조 보고서 "이후의 목표 (2) 하위 제어기의 제작 및 설치" 에 해당합니다.
처리 순서는 [AI_Priority_Rules/10] 2절 Step 3~5 를 따릅니다.

  목표 관절각(10) ─┬─ 워치독 (목표 미수신 timeout: [인터페이스 문서] IF-04 의 100 ms)
                   ├─ Tactile Latch (접촉 F ≥ 0.15 N 시 접촉 각 + 1.4° 로 래칭)   [AI_Priority_Rules/10 2.1]
                   ├─ 미끄러짐 반사 (slip_detected 시 텐던 목표 증분)            [인터페이스 문서 IF-05B, 4조 보고서 2.2]
                   ├─ 해제 인터락 (텐던 슬랙 선행 -> MCP 원복)                    [AI_Priority_Rules/10 2.1, ADR 005]
                   ├─ 속도 제한 (Slew)
                   └─ 서보 지령 변환 (STS3215 각, 텐던 서보 horn 각 = L(θ)/r)     [kinematics.py]

배열 순서는 모두 hand_spec.ACTUATOR_NAMES (10) 입니다.
촉각 배열(normal_forces[5], slip_detected[5]) 순서는 hand_spec.FINGERS (엄지..새끼) 입니다.
"""
import math
from dataclasses import dataclass

from . import hand_spec as H
from .kinematics import TendonParams, tendon_servo_angle

# 액추에이터 슬롯 -> 촉각 손가락 인덱스 (0 엄지 .. 4 새끼)
SLOT_FINGER = tuple(range(H.N_STS)) + tuple(range(H.N_TENDON))


@dataclass
class ControllerParams:
    dt: float = 0.01                    # 100 Hz [인터페이스 문서]
    watchdog_timeout: float = 0.10      # s [IF-04]
    watchdog_action: str = "hold"       # "hold" | "torque_off"
    max_vel_sts: float = 4.7            # rad/s [추정: 레포 v8/config.py STS_VMAX]
    max_vel_tendon: float = 4.0         # rad/s (PIP 관절 기준) [추정]
    latch_force: float = 0.15           # N [AI_Priority_Rules/10 2.1 Tactile Latch]
    latch_margin: float = math.radians(1.4)
    slip_step: float = math.radians(2.0)   # slip 1회당 PIP 목표 증분 [추정: 4조 1축 지그 실측 후 교체]
    slip_max_extra: float = math.radians(15.0)
    force_limit: float = 3.0            # N, 이 힘을 넘으면 더 조이지 않음 [추정: 레포 v8 F_TH_RANGE 상한]
    release_eps: float = math.radians(2.0)
    q_min: float = H.Q_MIN
    q_max: float = H.Q_MAX


class LowLevelController:
    def __init__(self, p: ControllerParams = None, tendon: TendonParams = None):
        self.p = p or ControllerParams()
        self.tp = tendon or TendonParams()
        self.cmd = [0.0] * H.N_ACTUATORS          # 현재 관절 공간 지령
        self.target = [0.0] * H.N_ACTUATORS
        self.t_target = None                      # 마지막 목표 수신 시각
        self.forces = [0.0] * 5
        self.slip = [False] * 5
        self.latch = [None] * H.N_ACTUATORS       # 래칭 상한 (rad) 또는 None
        self.slip_extra = [0.0] * 5               # 손가락별 텐던 추가 증분 (엄지..새끼)
        self.torque_enabled = True
        self.state = "ok"

    # ------------------------------------------------------------------ 입력
    def set_target(self, q10, now):
        if len(q10) != H.N_ACTUATORS:
            raise ValueError(f"목표는 {H.N_ACTUATORS}개여야 합니다")
        self.target = [float(v) for v in q10]
        self.t_target = now

    def set_tactile(self, forces=None, slip=None):
        if forces is not None:
            self.forces = [float(v) for v in forces[:5]]
        if slip is not None:
            self.slip = [bool(v) for v in slip[:5]]

    # ------------------------------------------------------------------ 루프
    def tick(self, now):
        """반환: dict(joint_cmd[10], servo_cmd[10], torque_enabled, state)"""
        p = self.p
        if self.t_target is None or now - self.t_target > p.watchdog_timeout:
            self.state = "watchdog"
            if p.watchdog_action == "torque_off":
                self.torque_enabled = False
            return self._output()
        self.state = "ok"
        self.torque_enabled = True

        goal = list(self.target)
        self._tactile_latch(goal)
        self._slip_reflex(goal)
        goal = [min(max(v, p.q_min), p.q_max) for v in goal]
        self._release_interlock(goal)

        for i in range(H.N_ACTUATORS):
            vmax = p.max_vel_sts if i in H.STS_SLOTS else p.max_vel_tendon
            step = vmax * p.dt
            d = goal[i] - self.cmd[i]
            self.cmd[i] += max(-step, min(step, d))
        return self._output()

    def _tactile_latch(self, goal):
        p = self.p
        for i in range(H.N_ACTUATORS):
            f = SLOT_FINGER[i]
            touching = self.forces[f] >= p.latch_force
            if touching and self.latch[i] is None:
                self.latch[i] = self.cmd[i] + p.latch_margin
            elif not touching:
                self.latch[i] = None
            if self.latch[i] is not None and goal[i] > self.latch[i]:
                goal[i] = self.latch[i]       # 더 조이는 방향만 막고, 펴는 방향은 그대로 허용

    def _slip_reflex(self, goal):
        p = self.p
        for f in range(5):
            slot = H.TENDON_SLOTS[f]
            if self.slip[f] and self.forces[f] < p.force_limit:
                self.slip_extra[f] = min(self.slip_extra[f] + p.slip_step, p.slip_max_extra)
            if self.forces[f] < p.latch_force:
                self.slip_extra[f] = 0.0      # 접촉이 끊기면 증분 해제
            if self.slip_extra[f] > 0.0:
                base = goal[slot]
                goal[slot] = base + self.slip_extra[f]
                if self.latch[slot] is not None:
                    self.latch[slot] = max(self.latch[slot], goal[slot])
            if self.forces[f] >= p.force_limit:
                goal[slot] = min(goal[slot], self.cmd[slot])

    def _release_interlock(self, goal):
        """MCP 를 펴려 할 때, 텐던이 아직 감겨 있으면 MCP 를 붙잡아 둡니다."""
        for f in range(5):
            mcp, ten = H.STS_SLOTS[f], H.TENDON_SLOTS[f]
            opening = goal[mcp] < self.cmd[mcp]
            tendon_wound = self.cmd[ten] > goal[ten] + self.p.release_eps
            if opening and tendon_wound:
                goal[mcp] = self.cmd[mcp]

    def _output(self):
        servo = [self.cmd[i] for i in H.STS_SLOTS]
        for f in range(5):
            mcp = self.cmd[H.STS_SLOTS[f]]
            pip = self.cmd[H.TENDON_SLOTS[f]]
            servo.append(tendon_servo_angle(mcp, pip, self.tp, thumb=(f == 0)))
        return dict(
            joint_cmd=list(self.cmd),
            servo_cmd=servo,
            torque_enabled=self.torque_enabled,
            state=self.state,
        )
