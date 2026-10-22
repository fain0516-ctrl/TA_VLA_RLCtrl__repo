#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/low_level_controller_node: 100 Hz 하위 제어기 (controller.LowLevelController 의 ROS 래퍼).

구독  /control/target_joint_positions  std_msgs/Float32MultiArray [10]   (Motion Mapping 또는 RL 정책)
      /tactile/normal_forces           std_msgs/Float32MultiArray [5]  N (4조, 엄지..새끼)
      /tactile/slip_detected           std_msgs/UInt8MultiArray   [5]    (4조, [IF-05B])
발행  /control/actuator_commands       std_msgs/Float32MultiArray [11]  servo_cmd[10] + torque_enabled(0/1)
      /control/joint_position_command  std_msgs/Float32MultiArray [14]  14 관절 지령 (RViz·기록용)
      /control/state                   std_msgs/String                  "ok" | "watchdog"
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, UInt8MultiArray, String

from . import hand_spec as H
from .controller import ControllerParams, LowLevelController
from .kinematics import TendonParams


class LowLevelControllerNode(Node):
    def __init__(self):
        super().__init__("low_level_controller_node")
        cp, tp = ControllerParams(), TendonParams()
        for k, v in vars(cp).items():
            self.declare_parameter(f"controller.{k}", v)
        for k, v in vars(tp).items():
            self.declare_parameter(f"tendon.{k}", v)
        cp = ControllerParams(**{k: self.get_parameter(f"controller.{k}").value for k in vars(cp)})
        tp = TendonParams(**{k: self.get_parameter(f"tendon.{k}").value for k in vars(tp)})
        self.ctrl = LowLevelController(cp, tp)

        fast = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32MultiArray, "/control/target_joint_positions", self.on_target, 10)
        self.create_subscription(Float32MultiArray, "/tactile/normal_forces", self.on_forces, fast)
        self.create_subscription(UInt8MultiArray, "/tactile/slip_detected", self.on_slip, 10)
        self.pub_act = self.create_publisher(Float32MultiArray, "/control/actuator_commands", 10)
        self.pub_q14 = self.create_publisher(Float32MultiArray, "/control/joint_position_command", 10)
        self.pub_state = self.create_publisher(String, "/control/state", 10)
        self._last_state = None
        self.create_timer(cp.dt, self.on_tick)
        self.get_logger().info(f"하위 제어기 시작 ({1.0 / cp.dt:.0f} Hz)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_target(self, msg):
        if len(msg.data) != H.N_ACTUATORS:
            self.get_logger().warn(f"목표 차원 오류: {len(msg.data)} (필요 {H.N_ACTUATORS})",
                                   throttle_duration_sec=2.0)
            return
        self.ctrl.set_target(list(msg.data), self.now())

    def on_forces(self, msg):
        if len(msg.data) >= 5:
            self.ctrl.set_tactile(forces=list(msg.data))

    def on_slip(self, msg):
        if len(msg.data) >= 5:
            self.ctrl.set_tactile(slip=list(msg.data))

    def on_tick(self):
        out = self.ctrl.tick(self.now())
        act = [float(v) for v in out["servo_cmd"]] + [1.0 if out["torque_enabled"] else 0.0]
        self.pub_act.publish(Float32MultiArray(data=act))
        q14 = H.actuators_to_joints(out["joint_cmd"], self.ctrl.tp.ratio)
        self.pub_q14.publish(Float32MultiArray(data=[float(v) for v in q14]))
        if out["state"] != self._last_state:
            self._last_state = out["state"]
            self.pub_state.publish(String(data=out["state"]))
            if out["state"] == "watchdog":
                self.get_logger().warn("목표 미수신: 워치독 동작 (%s)" % self.ctrl.p.watchdog_action)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControllerNode()
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
