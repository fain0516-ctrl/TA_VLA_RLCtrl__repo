#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/servo_bus_node: 서보 지령을 시뮬레이션 또는 실제 버스로 보내고 측정값을 발행합니다.

backend:=sim       서보 1차 지연 모델 (하드웨어 없이 파이프라인 검증)
backend:=pico      Pico 2 자작 보드 (W5500 UDP). 보드가 STS3215 x6 와 DS3225 x4 를 구동  ← 실기 기본값
backend:=hardware  Pi 직결: STS3215 x6 (/dev/ttyUSB0, 1 Mbps) + DS3225 x4 (PCA9685, /dev/i2c-1)

구독  /control/actuator_commands   std_msgs/Float32MultiArray [11]  servo_cmd[10] + torque_enabled
발행  /hardware/servo_positions    std_msgs/Float32MultiArray [10]  rad (텐던 서보는 지령 에코)
      /hardware/servo_load         std_msgs/Float32MultiArray [10]  STS 부하 비율 (-1~1), 텐던은 0
      /joint_states                sensor_msgs/JointState [14]     측정 기반 14 관절 (RViz2)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from . import hand_spec as H
from .backends import HardwareBackend, PicoBackend, SimBackend
from .kinematics import TendonParams, pip_from_servo_angle
from .pca9685 import ServoPulse


class ServoBusNode(Node):
    def __init__(self):
        super().__init__("servo_bus_node")
        d = self.declare_parameter
        d("backend", "sim")
        d("rate_hz", 100.0)
        d("sim.tau", 0.03)
        d("sim.vmax", 5.0)
        d("pico.ip", "192.168.10.20")                # [확인 필요] Pico W5500 고정 IP
        d("pico.cmd_port", 5556)
        d("pico.state_port", 5555)
        d("sts.port", "/dev/ttyUSB0")
        d("sts.ids", [1, 2, 3, 4, 5, 6])              # [인터페이스 문서] IF-02 모터 ID 1~10
        d("sts.zero_raw", [2048] * 6)
        d("sts.direction", [1] * 6)
        d("pca.bus", 1)
        d("pca.address", 0x40)
        d("pca.freq_hz", 50.0)
        d("pca.channels", [0, 1, 2, 3])
        d("pca.min_us", 500.0)
        d("pca.max_us", 2500.0)
        d("pca.range_rad", 4.712389)
        d("pca.zero_rad", [0.0] * 4)
        d("pca.direction", [1] * 4)
        for k, v in vars(TendonParams()).items():
            d(f"tendon.{k}", v)
        g = lambda n: self.get_parameter(n).value

        self.tp = TendonParams(**{k: g(f"tendon.{k}") for k in vars(TendonParams())})
        self.dt = 1.0 / float(g("rate_hz"))
        pulses = [ServoPulse(g("pca.min_us"), g("pca.max_us"), g("pca.range_rad"), z, s)
                  for z, s in zip(g("pca.zero_rad"), g("pca.direction"))]
        if g("backend") == "pico":
            self.bk = PicoBackend(g("pico.ip"), g("sts.zero_raw"), g("sts.direction"), pulses,
                                  g("pico.cmd_port"), g("pico.state_port"))
        elif g("backend") == "hardware":
            self.bk = HardwareBackend(g("sts.port"), g("sts.ids"), g("sts.zero_raw"), g("sts.direction"),
                                      g("pca.channels"), pulses, g("pca.bus"), g("pca.address"),
                                      g("pca.freq_hz"))
        else:
            self.bk = SimBackend(g("sim.tau"), g("sim.vmax"))
        self.get_logger().info(f"서보 백엔드: {g('backend')}")

        self.create_subscription(Float32MultiArray, "/control/actuator_commands", self.on_cmd, 10)
        self.pub_pos = self.create_publisher(Float32MultiArray, "/hardware/servo_positions", 10)
        self.pub_load = self.create_publisher(Float32MultiArray, "/hardware/servo_load", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(self.dt, self.on_tick)

    def on_cmd(self, msg):
        if len(msg.data) < H.N_ACTUATORS + 1:
            return
        self.bk.write(list(msg.data[:H.N_ACTUATORS]), msg.data[H.N_ACTUATORS] > 0.5)

    def on_tick(self):
        self.bk.step(self.dt)
        pos, load = self.bk.read()
        self.pub_pos.publish(Float32MultiArray(data=[float(v) for v in pos]))
        self.pub_load.publish(Float32MultiArray(data=[float(v) for v in load]))
        act = list(pos[:6]) + [pip_from_servo_angle(pos[6 + k], pos[2 + k], self.tp) for k in range(4)]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(H.JOINT_NAMES)
        js.position = [float(v) for v in H.actuators_to_joints(act, self.tp.ratio)]
        self.pub_js.publish(js)

    def destroy_node(self):
        self.bk.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoBusNode()
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
