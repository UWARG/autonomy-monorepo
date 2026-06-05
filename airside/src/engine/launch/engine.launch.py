from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="wrapper",
                executable="camera",
                name="camera_node",
                output="screen",
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="screen",
            ),
        ]
    )
