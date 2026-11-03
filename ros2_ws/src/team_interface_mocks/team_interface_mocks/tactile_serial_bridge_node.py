#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/tactile_serial_bridge_node: 4조 Arduino Uno FSR 값을 ROS 2 토픽으로 올리는 브릿지.

Arduino 는 한 줄에 손가락 5개의 ADC 값(0~1023)을 쉼표로 보낸다고 가정합니다 [제안 형식, 4조와 합의 필요]:
    "512,0,0,130,0\\n"     (엄지, 검지, 중지, 약지, 새끼)
센서가 1개뿐인 1축 지그 단계에서는 값 1개만 보내도 되며, 나머지 손가락은 0 N 으로 채웁니다.

발행  /tactile/normal_forces   std_msgs/Float32MultiArray [5]  N
      /tactile/slip_detected   std_msgs/UInt8MultiArray   [5]
      /tactile/contact         std_msgs/UInt8MultiArray   [5]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt8MultiArray

from .fsr import FsrParams, SlipDetector, adc_to_voltage, voltage_to_force


def parse_line(line, n=5):
    vals = [int(float(v)) for v in line.strip().split(",") if v.strip()]
    if not vals:
        raise ValueError("빈 줄")
    return (vals + [0] * n)[:n]


class TactileSerialBridge(Node):
    def __init__(self):
        super().__init__("tactile_serial_bridge_node")
        d = self.declare_parameter
        d("port", "/dev/ttyACM0")
        d("baudrate", 115200)
        fp = FsrParams()
        for k, v in vars(fp).items():
            d(f"fsr.{k}", v)
        g = lambda n: self.get_parameter(n).value
        self.fp = FsrParams(**{k: g(f"fsr.{k}") for k in vars(fp)})
        self.det = SlipDetector(self.fp)
        import serial
        self.ser = serial.Serial(g("port"), int(g("baudrate")), timeout=0)
        self.buf = b""
        self.pub_f = self.create_publisher(Float32MultiArray, "/tactile/normal_forces", 10)
        self.pub_s = self.create_publisher(UInt8MultiArray, "/tactile/slip_detected", 10)
        self.pub_c = self.create_publisher(UInt8MultiArray, "/tactile/contact", 10)
        self.create_timer(0.005, self.on_poll)

    def on_poll(self):
        self.buf += self.ser.read(256)
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            try:
                adc = parse_line(line.decode(errors="ignore"))
            except ValueError:
                continue
            forces = [voltage_to_force(adc_to_voltage(a, self.fp), self.fp) for a in adc]
            contact, slip = self.det.update(forces)
            self.pub_f.publish(Float32MultiArray(data=forces))
            self.pub_s.publish(UInt8MultiArray(data=[int(s) for s in slip]))
            self.pub_c.publish(UInt8MultiArray(data=[int(c) for c in contact]))

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TactileSerialBridge()
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
