# -*- coding: utf-8 -*-
"""Raspberry Pi 5 실기 구동. 1조·4조 실제 노드가 같은 토픽을 발행한다고 가정합니다.

    ros2 launch hand_motion_control hardware_bringup.launch.py
    ros2 launch hand_motion_control hardware_bringup.launch.py use_mock_perception:=true use_tactile_bridge:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("hand_motion_control"), "config", "hand_params.yaml")
    mock = LaunchConfiguration("use_mock_perception")
    bridge = LaunchConfiguration("use_tactile_bridge")
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_perception", default_value="false"),
        DeclareLaunchArgument("use_tactile_bridge", default_value="false"),
        DeclareLaunchArgument("tactile_port", default_value="/dev/ttyACM0"),
        Node(package="team_interface_mocks", executable="mock_hand_perception_node", condition=IfCondition(mock)),
        Node(package="team_interface_mocks", executable="tactile_serial_bridge_node", condition=IfCondition(bridge),
             parameters=[{"port": LaunchConfiguration("tactile_port")}]),
        Node(package="hand_motion_control", executable="motion_mapping_node", parameters=[params]),
        Node(package="hand_motion_control", executable="low_level_controller_node", parameters=[params]),
        Node(package="hand_motion_control", executable="servo_bus_node",
             parameters=[params, {"backend": "hardware"}]),
    ])
