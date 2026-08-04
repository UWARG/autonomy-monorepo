import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Local UDP endpoint where MAVROS mirrors the FCU stream (gcs_url) and
# where the RC bridge listens for RC_CHANNELS.
_RC_BRIDGE_PORT = 14550
_MAVROS_GCS_URL = f"udp://@127.0.0.1:{_RC_BRIDGE_PORT}"
_RC_BRIDGE_MAVLINK_URL = f"udpin:127.0.0.1:{_RC_BRIDGE_PORT}"


def generate_launch_description() -> LaunchDescription:
    mavros_distance_sensor_yaml = os.path.join(
        get_package_share_directory("engine"),
        "config",
        "mavros_distance_sensor.yaml",
    )

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
                        "gcs_url": _MAVROS_GCS_URL,
                        "fcu_protocol": "v2.0",
                        "tgt_system": 1,
                        "tgt_component": 1,
                        "plugin_denylist": ["rc_io"],
                    },
                    mavros_distance_sensor_yaml,
                ],
            ),
            Node(
                package="engine",
                executable="rc_bridge",
                name="rc_bridge",
                output="both",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"mavlink_url": _RC_BRIDGE_MAVLINK_URL}],
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
                package="wrapper",
                executable="camera",
                name="camera_node",
                output="both",
            ),
            Node(
                package="wrapper",
                executable="map_manager",
                name="map_manager_node",
                output="screen",
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="both",
            ),
        ]
    )
