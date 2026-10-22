# -*- coding: utf-8 -*-
"""노드 배선 검사: 가짜 rclpy 로 노드를 만들고 콜백을 직접 호출합니다."""
import fake_ros

fake_ros.install()

from hand_motion_control import hand_spec as H  # noqa: E402
from hand_motion_control.low_level_controller_node import LowLevelControllerNode  # noqa: E402
from hand_motion_control.motion_mapping_node import MotionMappingNode  # noqa: E402
from hand_motion_control.servo_bus_node import ServoBusNode  # noqa: E402
from team_interface_mocks.mock_hand_perception_node import MockHandPerception  # noqa: E402
from team_interface_mocks.mock_tactile_node import MockTactile  # noqa: E402


def _tick(node, k=0):
    node.timers[k][1]()


def test_end_to_end_sim_pipeline():
    per, mapn, ctl, bus, tac = (MockHandPerception(), MotionMappingNode(), LowLevelControllerNode(),
                                ServoBusNode(), MockTactile())
    assert "/hand_perception/joint_angles" in mapn.subs
    assert "/control/target_joint_positions" in ctl.subs and "/tactile/slip_detected" in ctl.subs
    t = 0.0
    for step in range(300):                       # 3 s @ 100 Hz
        t = step * 0.01
        for n in (per, mapn, ctl, bus, tac):
            n._clock.t = t
        if step % 3 == 0:
            _tick(per)
            mapn.subs["/hand_perception/joint_angles"](per.pubs["/hand_perception/joint_angles"].msgs[-1])
            ctl.subs["/control/target_joint_positions"](mapn.pubs["/control/target_joint_positions"].msgs[-1])
        _tick(ctl)
        bus.subs["/control/actuator_commands"](ctl.pubs["/control/actuator_commands"].msgs[-1])
        _tick(bus)
        tac.subs["/joint_states"](bus.pubs["/joint_states"].msgs[-1])
        _tick(tac)
        ctl.subs["/tactile/normal_forces"](tac.pubs["/tactile/normal_forces"].msgs[-1])
        ctl.subs["/tactile/slip_detected"](tac.pubs["/tactile/slip_detected"].msgs[-1])

    js = bus.pubs["/joint_states"].msgs[-1]
    assert js.name == list(H.JOINT_NAMES) and len(js.position) == 14
    assert max(js.position) > 0.3                 # 쥐는 동작이 실제로 전달됨
    act = ctl.pubs["/control/actuator_commands"].msgs[-1].data
    assert len(act) == 11 and act[-1] == 1.0
    assert ctl.ctrl.state == "ok"


def test_mapping_rejects_nothing_and_controller_rejects_bad_dim():
    ctl = LowLevelControllerNode()
    from fake_ros import Float32MultiArray
    ctl.subs["/control/target_joint_positions"](Float32MultiArray([0.0] * 3))
    assert ctl.ctrl.t_target is None
    assert any(level == "warn" for level, _ in ctl.get_logger().lines)
