# -*- coding: utf-8 -*-
"""rclpy·메시지 최소 대역 (테스트 전용). 실제 ROS 2 가 설치돼 있으면 쓰지 않습니다.

install() 는 sys.modules 에 rclpy, rclpy.node, rclpy.qos, std_msgs.msg, sensor_msgs.msg 를 넣습니다.
Node 는 publisher/subscription/timer 를 기록만 하고, 테스트가 콜백을 직접 호출합니다.
"""
import sys
import types


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Float32MultiArray(_Msg):
    def __init__(self, data=None):
        self.data = list(data or [])


class UInt8MultiArray(Float32MultiArray):
    pass


class String(_Msg):
    def __init__(self, data=""):
        self.data = data


class _Stamp:
    pass


class _Header:
    def __init__(self):
        self.stamp = _Stamp()
        self.frame_id = ""


class JointState(_Msg):
    def __init__(self):
        self.header = _Header()
        self.name, self.position, self.velocity, self.effort = [], [], [], []


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m, **_):
        self.lines.append(("info", m))

    def warn(self, m, **_):
        self.lines.append(("warn", m))

    error = warn


class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def to_msg(self):
        return _Stamp()


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return _Time(int(self.t * 1e9))


class Node:
    overrides = {}

    def __init__(self, name):
        self.name = name
        self._params = {}
        self.pubs, self.subs, self.timers = {}, {}, []
        self._logger = _Logger()
        self._clock = _Clock()

    def declare_parameter(self, name, value=None):
        self._params[name] = Node.overrides.get(name, value)
        return _Param(self._params[name])

    def get_parameter(self, name):
        return _Param(self._params[name])

    def create_publisher(self, _t, topic, _q):
        self.pubs[topic] = _Pub(topic)
        return self.pubs[topic]

    def create_subscription(self, _t, topic, cb, _q):
        self.subs[topic] = cb

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


def install():
    if "rclpy" in sys.modules and not getattr(sys.modules["rclpy"], "_fake", False):
        return
    rclpy = types.ModuleType("rclpy")
    rclpy._fake = True
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda n: None
    node = types.ModuleType("rclpy.node")
    node.Node = Node
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: kw
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    std = types.ModuleType("std_msgs.msg")
    std.Float32MultiArray, std.UInt8MultiArray, std.String = Float32MultiArray, UInt8MultiArray, String
    sen = types.ModuleType("sensor_msgs.msg")
    sen.JointState = JointState
    sys.modules.update({
        "rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos,
        "std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std,
        "sensor_msgs": types.ModuleType("sensor_msgs"), "sensor_msgs.msg": sen,
    })
