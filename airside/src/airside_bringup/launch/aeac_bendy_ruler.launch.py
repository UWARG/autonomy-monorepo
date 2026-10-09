"""Live AEAC traffic into the existing BendyRuler2D lapping flight stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    base_launch = Path(get_package_share_directory("airside_bringup")) / "launch"
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch / "airside.launch.py")),
                launch_arguments={
                    "obstacle_source": "traffic",
                    "traffic_topic": "/aeac/live_traffic",
                    "horizontal_speed_mps": "0.8",
                }.items(),
            ),
            Node(
                package="comms",
                executable="traffic_listener",
                name="traffic_listener",
                output="both",
            ),
            Node(
                package="comms",
                executable="telemetry_sender",
                name="aeac_telemetry_sender",
                output="both",
            ),
        ]
    )
