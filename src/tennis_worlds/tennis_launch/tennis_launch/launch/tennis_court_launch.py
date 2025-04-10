import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('tennis_launch')
    world_path = os.path.join(pkg_dir, 'worlds', 'tennis_court.world')

    # 加载gazebo_ros带world的launch文件（带gzserver和gzclient）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # 生成TurtleBot3实体（需要TURTLEBOT3_MODEL=burger，且模型数据库中有）
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',
            '-database', 'turtlebot3_burger',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_tb3
    ])
