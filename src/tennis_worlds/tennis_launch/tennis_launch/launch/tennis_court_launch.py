#!/usr/bin/env python3
import os, random, tempfile
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory   # ←① 新增

TB3_MODEL_PATH = '/opt/ros/humble/share/turtlebot3_gazebo/models'
NUM_BALLS, BALL_RANGE = 5, 4.0

# ② 去掉开头多余换行
BALL_SDF_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.056</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia>
      </inertial>
      <collision name='collision'>
        <geometry><sphere><radius>0.033</radius></sphere></geometry>
      </collision>
      <visual name='visual'>
        <geometry><sphere><radius>0.033</radius></sphere></geometry>
        <material><ambient>0 1 0 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn_ball(i: int) -> Node:
    name = f'tennis_ball_{i}'
    tmp = tempfile.NamedTemporaryFile(prefix=name, suffix='.sdf', delete=False)
    tmp.write(BALL_SDF_TEMPLATE.format(name=name).encode()); tmp.close()
    x, y = (random.uniform(-BALL_RANGE, BALL_RANGE) for _ in range(2))
    return Node(package='gazebo_ros', executable='spawn_entity.py',
                arguments=['-entity', name, '-file', tmp.name,
                           '-x', str(x), '-y', str(y), '-z', '0.05'],
                output='screen')

def generate_launch_description():
    # 环境变量：让 Gazebo 找到 TurtleBot3 模型
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        f'{TB3_MODEL_PATH}:{os.environ.get("GAZEBO_MODEL_PATH", "")}')

    # ★ ① 正确获取 tennis_launch 安装目录
    pkg_share = get_package_share_directory('tennis_launch')
    world_path = os.path.join(pkg_share, 'worlds', 'tennis_court.world')

    gazebo = ExecuteProcess(
    cmd=[
        'gazebo',
        '--verbose',
        str(world_path),
        '-s', 'libgazebo_ros_factory.so',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_api_plugin.so'  # 🔥 重点插件
    ],
    output='screen'
  )

    spawn_tb3 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle_pi',
                   '-database', 'turtlebot3_waffle_pi',
                   '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen')

    balls = [spawn_ball(i) for i in range(NUM_BALLS)]
    return LaunchDescription([set_model_path, gazebo, spawn_tb3] + balls)
