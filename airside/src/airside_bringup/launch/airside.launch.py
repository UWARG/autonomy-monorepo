from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Local UDP endpoint where MAVROS mirrors the FCU stream (gcs_url) and
# where the RC bridge listens for RC_CHANNELS.
_RC_BRIDGE_PORT = 14550
_MAVROS_GCS_URL = f"udp://@127.0.0.1:{_RC_BRIDGE_PORT}"
_RC_BRIDGE_MAVLINK_URL = f"udpin:127.0.0.1:{_RC_BRIDGE_PORT}"


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
            DeclareLaunchArgument(
                "obstacle_source",
                default_value=EnvironmentVariable(
                    "OBSTACLE_SOURCE", default_value="traffic"
                ),
                description="Exactly one obstacle source: traffic or scan",
            ),
            DeclareLaunchArgument(
                "aeac_url",
                default_value=EnvironmentVariable("AEAC_URL", default_value=""),
            ),
            DeclareLaunchArgument(
                "aeac_token",
                default_value=EnvironmentVariable("AEAC_TOKEN", default_value=""),
            ),
            DeclareLaunchArgument(
                "aeac_uav_id",
                default_value=EnvironmentVariable("AEAC_UAV_ID", default_value=""),
            ),
            DeclareLaunchArgument(
                "aeac_own_aircraft_index",
                default_value=EnvironmentVariable(
                    "AEAC_OWN_AIRCRAFT_INDEX", default_value="-1"
                ),
            ),
            DeclareLaunchArgument(
                "aeac_protocol_verified",
                default_value=EnvironmentVariable(
                    "AEAC_PROTOCOL_VERIFIED", default_value="false"
                ),
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
                    }
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
                package="camera",
                executable="camera",
                name="camera_node",
                output="both",
            ),
            Node(
                package="camera",
                executable="triggered_image_publisher",
                name="triggered_image_publisher",
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
                package="aeac_bridge",
                executable="bridge",
                name="aeac_bridge",
                output="both",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "aeac_websocket_url": LaunchConfiguration("aeac_url"),
                        "aeac_connection_token": LaunchConfiguration("aeac_token"),
                        "uav_id": LaunchConfiguration("aeac_uav_id"),
                        "own_aircraft_index": ParameterValue(
                            LaunchConfiguration("aeac_own_aircraft_index"),
                            value_type=int,
                        ),
                        "protocol_verified": ParameterValue(
                            LaunchConfiguration("aeac_protocol_verified"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="both",
                parameters=[
                    {
                        "obstacle_avoidance.source": LaunchConfiguration(
                            "obstacle_source"
                        )
                    }
                ],
            ),
        ]
    )
