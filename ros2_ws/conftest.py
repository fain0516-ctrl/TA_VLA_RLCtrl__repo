# -*- coding: utf-8 -*-
"""ROS 2 없이 pytest 로 패키지 로직과 노드 배선을 검사합니다: python -m pytest ros2_ws -q"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for pkg in ("hand_motion_control", "team_interface_mocks"):
    sys.path.insert(0, os.path.join(HERE, "src", pkg))
sys.path.insert(0, os.path.join(HERE, "test_support"))
