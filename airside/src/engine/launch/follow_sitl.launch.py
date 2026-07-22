"""SITL follow profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    source = os.path.join(
        get_package_share_directory("engine"), "launch", "follow.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="tcp://127.0.0.1:5762"),
            DeclareLaunchArgument("world_target", default_value="true"),
            DeclareLaunchArgument("lunge", default_value="false"),
            DeclareLaunchArgument("crossing", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(source),
                launch_arguments={
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "sim_target": "true",
                    "world_target": LaunchConfiguration("world_target"),
                    "lunge": LaunchConfiguration("lunge"),
                    "crossing": LaunchConfiguration("crossing"),
                    "props_off_hitl": "false",
                }.items(),
            ),
            Node(package="engine", executable="sitl_handoff", output="both"),
        ]
    )
