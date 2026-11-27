import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('global_planner'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='global_planner',
            executable='global_planner_node',
            name='global_planner',
            parameters=[params_file],
            output='screen',
            emulate_tty=True,
        )
    ])
