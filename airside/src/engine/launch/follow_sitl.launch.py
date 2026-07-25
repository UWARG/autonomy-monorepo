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
            DeclareLaunchArgument("detector_stride", default_value="1"),
            DeclareLaunchArgument("sim_latency_s", default_value="0.0"),
            DeclareLaunchArgument("occlusion_after_s", default_value="-1.0"),
            DeclareLaunchArgument("occlusion_duration_s", default_value="0.0"),
            DeclareLaunchArgument("drop_detector_every_n", default_value="0"),
            DeclareLaunchArgument("timing_json", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(source),
                launch_arguments={
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "sim_target": "true",
                    "world_target": LaunchConfiguration("world_target"),
                    "lunge": LaunchConfiguration("lunge"),
                    "crossing": LaunchConfiguration("crossing"),
                    "detector_stride": LaunchConfiguration("detector_stride"),
                    "sim_latency_s": LaunchConfiguration("sim_latency_s"),
                    "occlusion_after_s": LaunchConfiguration("occlusion_after_s"),
                    "occlusion_duration_s": LaunchConfiguration("occlusion_duration_s"),
                    "drop_detector_every_n": LaunchConfiguration(
                        "drop_detector_every_n"
                    ),
                    "timing_json": LaunchConfiguration("timing_json"),
                    "props_off_hitl": "false",
                }.items(),
            ),
            Node(package="engine", executable="sitl_handoff", output="both"),
        ]
    )
