# -*- coding: utf-8 -*-
"""hand_motion_control 핵심 로직 (ROS 불필요)."""
import math

import pytest

from hand_motion_control import hand_spec as H
from hand_motion_control import sts_protocol as S
from hand_motion_control.controller import ControllerParams, LowLevelController
from hand_motion_control.kinematics import (TendonParams, finger_tip, pip_from_servo_angle,
                                            tendon_servo_angle)
from hand_motion_control.mapping import MappingParams, map_human_to_robot, project_pip
from hand_motion_control.pca9685 import ServoPulse, prescale_for, us_to_ticks


def test_spec_sizes_and_coupling():
    assert H.N_JOINTS == 14 and H.N_ACTUATORS == 10
    q = H.actuators_to_joints([0.1, 0.2, 0.3, 0.3, 0.3, 0.3, 1.0, 1.0, 1.0, 1.0])
    assert q[H.joint_index(1, 2)] == pytest.approx(0.8)
    assert H.JOINT_NAMES[H.joint_index(4, 2)] == "joint_4_2"


def test_fk_straight_finger_length():
    x, z = finger_tip(0.0, 0.0)
    assert x == pytest.approx(H.L_PROX + H.L_MID + H.L_DIST) and z == pytest.approx(0.0)


def test_tendon_roundtrip_and_mcp_decoupling():
    tp = TendonParams()
    a = tendon_servo_angle(0.5, 1.0, tp)
    assert pip_from_servo_angle(a, 0.5, tp) == pytest.approx(1.0)
    # MCP 만 굽혀도 텐던 길이가 변하므로 horn 각이 커져야 함 (ΔL = r_mcp·Δq_mcp)
    assert tendon_servo_angle(0.5, 0.0, tp) == pytest.approx(0.5 * tp.r_mcp / tp.r_pulley)


def test_project_pip_on_coupling_line():
    assert project_pip(1.0, 0.8, 0.8) == pytest.approx(1.0)
    assert project_pip(1.0, None, 0.8) == 1.0


def test_mapping_full_and_missing():
    p = MappingParams()
    human = {"thumb_mcp": 0.4, "thumb_ip": 0.5, "index_mcp": 0.3, "index_pip": 1.0, "index_dip": 0.8}
    out, missing = map_human_to_robot(human, p)
    assert out[0] == pytest.approx(0.4) and out[2] == pytest.approx(0.3) and out[6] == pytest.approx(1.0)
    assert "middle_mcp" in missing and len(out) == 10
    out2, _ = map_human_to_robot({"index_mcp": 9.0}, p, previous=out)
    assert out2[2] == pytest.approx(H.Q_MAX)       # 절삭
    assert out2[0] == pytest.approx(0.4)          # 누락 관절은 이전 값 유지


def _run(ctrl, t0, n, dt=0.01):
    """목표를 매 tick 다시 보내며(워치독 회피) n tick 실행."""
    out, goal = None, list(ctrl.target)
    for k in range(n):
        ctrl.set_target(goal, t0 + k * dt)
        out = ctrl.tick(t0 + k * dt)
    return out


def test_controller_slew_and_watchdog():
    c = LowLevelController(ControllerParams())
    c.set_target([1.0] * 10, 0.0)
    out = c.tick(0.0)
    assert out["joint_cmd"][0] == pytest.approx(4.7 * 0.01)     # 한 tick 속도 제한
    out = c.tick(0.5)                                            # 0.1 s 넘게 목표 없음
    assert out["state"] == "watchdog" and out["torque_enabled"]
    c2 = LowLevelController(ControllerParams(watchdog_action="torque_off"))
    assert c2.tick(0.0)["torque_enabled"] is False              # 목표 수신 전엔 구동하지 않음


def test_controller_tactile_latch_blocks_closing_only():
    c = LowLevelController(ControllerParams())
    c.set_target([0.5] * 10, 0.0)
    _run(c, 0.0, 5)
    c.set_tactile(forces=[0.0, 0.2, 0.0, 0.0, 0.0])            # 검지 접촉
    before = c.cmd[6]
    for k in range(50):
        c.set_target([1.5] * 10, 0.05 + k * 0.01)
        out = c.tick(0.05 + k * 0.01)
    assert out["joint_cmd"][6] <= before + math.radians(1.4) + 1e-9
    assert out["joint_cmd"][7] > 1.0                            # 중지는 그대로 굽힘
    c.set_target([0.0] * 10, 1.0)
    out = c.tick(1.0)
    assert out["joint_cmd"][6] < before + math.radians(1.4)    # 펴는 방향은 허용


def test_controller_slip_reflex_tightens():
    c = LowLevelController(ControllerParams())
    c.set_target([0.5] * 10, 0.0)
    _run(c, 0.0, 20)
    c.set_tactile(forces=[0.0, 0.5, 0.0, 0.0, 0.0], slip=[0, 1, 0, 0, 0])
    c.set_target([0.5] * 10, 0.2)
    _run(c, 0.2, 5)
    assert c.cmd[6] > 0.5 + 1e-6


def test_controller_release_interlock():
    c = LowLevelController(ControllerParams())
    c.set_target([1.0] * 10, 0.0)
    _run(c, 0.0, 60)
    c.set_target([0.0] * 10, 0.6)
    out = c.tick(0.6)
    assert out["joint_cmd"][2] == pytest.approx(1.0)            # MCP 는 텐던이 풀릴 때까지 대기
    assert out["joint_cmd"][6] < 1.0
    out = _run(c, 0.61, 60)
    assert out["joint_cmd"][2] == pytest.approx(0.0, abs=1e-9)


def test_sts_packets():
    pkt = S.write_torque_enable(1, True)
    assert pkt == bytes([0xFF, 0xFF, 0x01, 0x04, 0x03, 40, 0x01, (~(1 + 4 + 3 + 40 + 1)) & 0xFF])
    sync = S.sync_write_goal_positions({1: 2048, 2: 4095})
    assert sync[2] == 0xFE and sync[4] == 0x83 and sync[5] == 42 and sync[6] == 2
    assert list(sync[7:13]) == [1, 0x00, 0x08, 2, 0xFF, 0x0F]
    body = [0x01, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x10, 0x04]
    sid, err, prm = S.parse_status(bytes([0xFF, 0xFF] + body + [S.checksum(body)]))
    assert sid == 1 and err == 0 and S.word(prm[0], prm[1]) == 2048
    assert S.signed_magnitude(S.word(prm[4], prm[5]), 10) == -16
    with pytest.raises(ValueError):
        S.parse_status(bytes([0xFF, 0xFF] + body + [0]))
    assert S.raw_to_rad(S.rad_to_raw(0.7)) == pytest.approx(0.7, abs=2 * math.pi / 4096)


def test_pca9685_math():
    assert prescale_for(50.0) == 121
    assert us_to_ticks(1500, 50.0) == 307
    sp = ServoPulse()
    assert sp.angle_to_us(0.0) == 500.0 and sp.angle_to_us(100.0) == 2500.0
