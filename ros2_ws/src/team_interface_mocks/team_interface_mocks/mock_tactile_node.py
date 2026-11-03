#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/mock_tactile_node: 4조 출력을 흉내내는 가상 물체 촉각 발행기 (시뮬레이션 폐루프용).

손가락 굴곡(MCP + PIP)이 contact_angle 을 넘으면 물체에 닿았다고 보고,
초과 각에 비례하는 법선력을 만듭니다. slip_period_s > 0 이면 주기적으로 미끄러짐을 주입합니다.

구독  /joint_states            sensor_msgs/JointState  (14 관절)
발행  /tactile/normal_forces   std_msgs/Float32MultiArray [5]  N   (엄지..새끼)
      /tactile/slip_detected   std_msgs/UInt8MultiArray   [5]
"""
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

FLEX_JOINTS = (
    ("joint_0_0", "joint_0_1"),
    ("joint_1_0", "joint_1_1"),
    ("joint_2_0", "joint_2_1"),
    ("joint_3_0", "joint_3_1"),
    ("joint_4_0", "joint_4_1"),
)


class MockTactile(Node):
    def __init__(self):
        super().__init__("mock_tactile_node")
        d = self.declare_parameter
        d("rate_hz", 100.0)
        d("contact_angle", 1.2)      # rad [추정: 가상 물체 크기]
        d("stiffness", 4.0)          # N/rad [추정]
        d("slip_period_s", 0.0)
        g = lambda n: self.get_parameter(n).value
        self.c_ang, self.k = float(g("contact_angle")), float(g("stiffness"))
        self.slip_period = float(g("slip_period_s"))
        self.dt = 1.0 / float(g("rate_hz"))
        self.q = {}
        self.t_slip = 0.0
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.pub_f = self.create_publisher(Float32MultiArray, "/tactile/normal_forces", 10)
        self.pub_s = self.create_publisher(UInt8MultiArray, "/tactile/slip_detected", 10)
        self.create_timer(self.dt, self.on_tick)

    def on_js(self, msg):
        self.q = dict(zip(msg.name, msg.position))

    def on_tick(self):
        forces = []
        for a, b in FLEX_JOINTS:
            flex = self.q.get(a, 0.0) + self.q.get(b, 0.0)
            forces.append(max(0.0, self.k * (flex - self.c_ang)))
        slip = [0] * 5
        if self.slip_period > 0.0:
            self.t_slip += self.dt
            touching = [i for i, f in enumerate(forces) if f > 0.0]
            if self.t_slip >= self.slip_period and touching:
                self.t_slip = 0.0
                slip[random.choice(touching)] = 1
        self.pub_f.publish(Float32MultiArray(data=[float(f) for f in forces]))
        self.pub_s.publish(UInt8MultiArray(data=slip))


def main(args=None):
    rclpy.init(args=args)
    node = MockTactile()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
