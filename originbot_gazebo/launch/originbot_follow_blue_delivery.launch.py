import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'originbot_gazebo'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, 'worlds', 'blue_delivery_city.world')

    originbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'originbot.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path, 'gui': 'false'}.items()
    )

    gazebo_client = ExecuteProcess(cmd=['gzclient'], output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'originbot',
            '-x', '0.3',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        originbot,
        gazebo_server,
        gazebo_client,
        spawn_entity,
    ])
