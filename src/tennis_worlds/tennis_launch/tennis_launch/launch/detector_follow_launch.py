#!/usr/bin/env python3
"""
detector_follow.launch.py
-------------------------
启动 YOLO 检测节点 + 球跟随节点
注意：Gazebo 仿真仍然用 tennis_court_launch.py 另开终端先启动
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 1) YOLO 检测节点
    detector = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            # 如需换成自己训练好的权重就在下面改
            {'weight_path': ''}   # 空串 → 用包内 yolov8n.pt
        ]
    )

    # 2) 简单跟随节点
    follower = Node(
        package='ball_follower_pkg',
        executable='ball_follower',
        name='ball_follower',
        parameters=[
            {'kp_ang': 1.2, 'linear_spd': 0.15}
        ]
    )

    return LaunchDescription([detector, follower])
