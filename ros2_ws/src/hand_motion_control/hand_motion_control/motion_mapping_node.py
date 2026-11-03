#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/motion_mapping_node: 1조 사람 손 관절각 -> 10 액추에이터 목표 관절각.

구독  /hand_perception/joint_angles   sensor_msgs/JointState   (1조, 이름+각도 rad)
발행  /control/target_joint_positions std_msgs/Float32MultiArray [10]  (hand_spec.ACTUATOR_NAMES 순서)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from . import hand_spec as H
from .mapping import DEFAULT_HUMAN_NAMES, MappingParams, map_human_to_robot


class MotionMappingNode(Node):
    def __init__(self):
        super().__init__("motion_mapping_node")
        self.declare_parameter("dip_pip_ratio", H.DIP_PIP_RATIO)
        self.declare_parameter("gain", [1.0] * H.N_ACTUATORS)
        self.declare_parameter("offset", [0.0] * H.N_ACTUATORS)
        for finger, names in DEFAULT_HUMAN_NAMES.items():
            self.declare_parameter(f"human_names.{finger}", list(names))

        g = lambda n: self.get_parameter(n).value
        self.p = MappingParams(
            ratio=float(g("dip_pip_ratio")),
            gain=[float(v) for v in g("gain")],
            offset=[float(v) for v in g("offset")],
            names={f: tuple(g(f"human_names.{f}")) for f in DEFAULT_HUMAN_NAMES},
        )
        self.prev = None
        self.pub = self.create_publisher(Float32MultiArray, "/control/target_joint_positions", 10)
        self.create_subscription(JointState, "/hand_perception/joint_angles", self.on_human, 10)
        self.get_logger().info("Motion Mapping 시작: /hand_perception/joint_angles -> /control/target_joint_positions")

    def on_human(self, msg: JointState):
        human = {n: float(q) for n, q in zip(msg.name, msg.position)}
        out, missing = map_human_to_robot(human, self.p, self.prev)
        if missing:
            self.get_logger().warn(f"1조 관절 누락: {missing}", throttle_duration_sec=2.0)
        self.prev = out
        self.pub.publish(Float32MultiArray(data=[float(v) for v in out]))


def main(args=None):
    rclpy.init(args=args)
    node = MotionMappingNode()
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
