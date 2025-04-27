#!/usr/bin/env python3
"""
只启动 YOLO 检测 和 BallFollower ，**不再包含 Gazebo**。

用法：
    # ① 终端 A 先开 Gazebo
    ros2 launch tennis_launch tennis_court_launch.py

    # ② 终端 B 再开检测+跟随
    ros2 launch tennis_launch detector_follow_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

BEST = '/mnt/d/Python_YOLO5_forTennisDetection/runs/detect/train12/weights/best.pt'

def generate_launch_description():
    yolo = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{'weight_path': BEST}]
    )

    follower = Node(
        package='ball_follower_pkg',
        executable='ball_follower',
        name='ball_follower',
        output='screen',
        parameters=[{
            'lin_spd':   0.15,
            'ang_spd':   0.8,
            'deadband':  40.0,
            'img_width': 640.0
        }]
    )

    return LaunchDescription([yolo, follower])
