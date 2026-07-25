"""Props-off follow HITL profile."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    source = os.path.join(
        get_package_share_directory("engine"), "launch", "follow.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value=EnvironmentVariable(
                    "FCU_URL", default_value="serial:///dev/serial0:115200"
                ),
            ),
            DeclareLaunchArgument("blob_path", default_value=""),
            DeclareLaunchArgument("person_label", default_value="15"),
            DeclareLaunchArgument("camera_fps", default_value="20"),
            DeclareLaunchArgument("detector_stride", default_value="1"),
            DeclareLaunchArgument("foxglove", default_value="false"),
            DeclareLaunchArgument("world_target", default_value="false"),
            DeclareLaunchArgument("lunge", default_value="false"),
            DeclareLaunchArgument("crossing", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(source),
                launch_arguments={
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "props_off_hitl": "true",
                    "sim_target": "false",
                    "oakd_target": "true",
                    "blob_path": LaunchConfiguration("blob_path"),
                    "person_label": LaunchConfiguration("person_label"),
                    "camera_fps": LaunchConfiguration("camera_fps"),
                    "detector_stride": LaunchConfiguration("detector_stride"),
                    "foxglove": LaunchConfiguration("foxglove"),
                }.items(),
            ),
        ]
    )
