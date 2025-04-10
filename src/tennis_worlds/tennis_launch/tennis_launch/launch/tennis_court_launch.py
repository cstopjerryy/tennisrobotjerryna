import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo with tennis_court.world and spawn a TurtleBot3 Waffle Pi that carries an RGB‑D camera by default.

    * 通过 `GAZEBO_MODEL_PATH` 指向 turtlebot3_gazebo/models，确保离线加载模型；
    * 使用 spawn_entity.py 从 Gazebo 模型数据库加载 waffle_pi；
    * 以后如需额外传感器，可在此文件继续追加 Node/ExecuteProcess。
    """

    # -----------------------------
    # 0. 环境变量：让 Gazebo 能找到 TurtleBot3 模型
    # -----------------------------
    tb3_models = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')
    # 若已有其他路径，追加；否则直接设置
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{tb3_models}:{gazebo_model_path}" if gazebo_model_path else tb3_models

    set_gazebo_env = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=new_model_path)

    # -----------------------------
    # 1.  World file
    # -----------------------------
    pkg_dir = get_package_share_directory('tennis_launch')
    world_file = os.path.join(pkg_dir, 'worlds', 'tennis_court.world')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_factory.so'  # 插件：允许运行时 spawn 实体
        ],
        output='screen'
    )

    # -----------------------------
    # 2.  Spawn TurtleBot3 Waffle Pi
    # -----------------------------
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3',                # 实体名
            '-database', 'turtlebot3_waffle_pi',
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_env,
        gazebo,
        spawn_tb3
    ])
