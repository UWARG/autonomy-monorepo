from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value=EnvironmentVariable(
                    "FCU_URL", default_value="serial:///dev/serial0:115200"
                ),
                description="MAVROS connection URL to ArduPilot",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="mavros",
                output="both",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "fcu_url": LaunchConfiguration("fcu_url"),
                        "fcu_protocol": "v2.0",
                        "tgt_system": 1,
                        "tgt_component": 1,
                    }
                ],
            ),
            Node(
                package="engine",
                executable="heartbeat",
                name="heartbeat_node",
                output="both",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="both",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"port": 9090}],
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="both",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="camera",
                executable="camera",
                name="camera_node",
                output="both",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="navigation",
                executable="position_controller",
                name="position_controller",
                output="both",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="both",
            ),
        ]
    )
