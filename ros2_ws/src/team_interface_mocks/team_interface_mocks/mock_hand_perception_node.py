#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/mock_hand_perception_node: 1조 출력을 흉내내는 mock data 발행기.

1조 보고서: 카메라를 항상 쓸 수 없어 mock data 로 개별 모듈을 먼저 개발하고 ROS2 Publisher/Subscriber
전달을 확인한다고 했습니다. 이 노드는 3조 쪽에서 같은 방식으로 입력을 만들어 줍니다.

발행  /hand_perception/joint_angles  sensor_msgs/JointState  (30 Hz, 쥐었다 폈다 반복 + 잡음)
"""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = [
    "thumb_mcp", "thumb_ip",
    "index_mcp", "index_pip", "index_dip",
    "middle_mcp", "middle_pip", "middle_dip",
    "ring_mcp", "ring_pip", "ring_dip",
    "pinky_mcp", "pinky_pip", "pinky_dip",
]
# 완전히 쥐었을 때 사람 손 관절각 rad [추정]
FULL = {"mcp": 1.3, "pip": 1.5, "dip": 1.0, "ip": 1.1}


class MockHandPerception(Node):
    def __init__(self):
        super().__init__("mock_hand_perception_node")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("period_s", 4.0)
        self.declare_parameter("noise_rad", 0.02)
        self.rate = float(self.get_parameter("rate_hz").value)
        self.period = float(self.get_parameter("period_s").value)
        self.noise = float(self.get_parameter("noise_rad").value)
        self.t = 0.0
        self.pub = self.create_publisher(JointState, "/hand_perception/joint_angles", 10)
        self.create_timer(1.0 / self.rate, self.on_tick)

    def on_tick(self):
        self.t += 1.0 / self.rate
        s = 0.5 - 0.5 * math.cos(2 * math.pi * self.t / self.period)   # 0(편 손) ~ 1(쥔 손)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.name = list(NAMES)
        msg.position = [max(0.0, s * FULL[n.split("_")[1]] + random.gauss(0.0, self.noise)) for n in NAMES]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockHandPerception()
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
