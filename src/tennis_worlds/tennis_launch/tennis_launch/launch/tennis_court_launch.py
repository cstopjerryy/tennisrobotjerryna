#!/usr/bin/env python3
"""
启动 Gazebo -> 加载 tennis_court.world
 -> 设置 GAZEBO_MODEL_PATH（让 waffle_pi 能被 -database 找到）
 -> Spawn TurtleBot3 Waffle Pi
 -> 随机生成 5 颗 tennis_ball_0..4
"""

import os, random, tempfile
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node

TB3_MODEL_PATH = '/opt/ros/humble/share/turtlebot3_gazebo/models'
NUM_BALLS      = 5          # 需要几颗球
BALL_RANGE     = 4.0        # 球散落范围（±米）

BALL_SDF_TEMPLATE = """
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='link'>
      <inertial>
        <mass>0.056</mass>
        <inertia>
          <ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
        </inertia>
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

def generate_ball_spawn_cmd(i):
    """写临时 SDF 并返回 spawn_entity Node"""
    name = f'tennis_ball_{i}'
    sdf  = BALL_SDF_TEMPLATE.format(name=name)
    tmp  = tempfile.NamedTemporaryFile(prefix=name, suffix='.sdf', delete=False)
    tmp.write(sdf.encode()); tmp.close()

    x = random.uniform(-BALL_RANGE, BALL_RANGE)
    y = random.uniform(-BALL_RANGE, BALL_RANGE)
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', name, '-file', tmp.name,
                   '-x', str(x), '-y', str(y), '-z', '0.05'],
        output='screen'
    )

def generate_launch_description():
    # 1) 环境变量：让 Gazebo 找到 turtlebot3_waffle_pi
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        f'{TB3_MODEL_PATH}:{os.environ.get("GAZEBO_MODEL_PATH", "")}'
    )

    # 2) 启动 Gazebo + 世界
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             os.path.join(
                 os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                 'share/tennis_launch/worlds/tennis_court.world'),
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3) Spawn TurtleBot3 Waffle Pi（自带相机）
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'turtlebot3_waffle_pi',
                   '-database', 'turtlebot3_waffle_pi',
                   '-x', '0', '-y', '0', '-z', '0.01'],
        output='screen'
    )

    # 4) 随机生成网球
    balls = [generate_ball_spawn_cmd(i) for i in range(NUM_BALLS)]

    return LaunchDescription(
        [set_model_path, gazebo, spawn_tb3] + balls
    )
