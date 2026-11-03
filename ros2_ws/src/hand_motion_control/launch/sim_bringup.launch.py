# -*- coding: utf-8 -*-
"""하드웨어 없이 전체 파이프라인을 돌립니다.

1조 mock -> Motion Mapping -> 하위 제어기 -> 서보(sim) -> /joint_states -> 4조 mock(가상 물체) -> 하위 제어기
    ros2 launch hand_motion_control sim_bringup.launch.py
    ros2 launch hand_motion_control sim_bringup.launch.py slip_period_s:=1.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("hand_motion_control"), "config", "hand_params.yaml")
    slip = LaunchConfiguration("slip_period_s")
    return LaunchDescription([
        DeclareLaunchArgument("slip_period_s", default_value="0.0"),
        Node(package="team_interface_mocks", executable="mock_hand_perception_node"),
        Node(package="team_interface_mocks", executable="mock_tactile_node",
             parameters=[{"slip_period_s": slip}]),
        Node(package="hand_motion_control", executable="motion_mapping_node", parameters=[params]),
        Node(package="hand_motion_control", executable="low_level_controller_node", parameters=[params]),
        Node(package="hand_motion_control", executable="servo_bus_node", parameters=[params, {"backend": "sim"}]),
    ])
