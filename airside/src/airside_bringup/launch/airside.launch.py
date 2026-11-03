from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
                    "FCU_URL", default_value="serial:///dev/ttyAMA0:115200"
                ),
                description="MAVROS connection URL to ArduPilot",
            ),
            DeclareLaunchArgument(
                "lap_count",
                default_value=EnvironmentVariable("LAP_COUNT", default_value="3"),
                description="Number of laps to fly before reconnaissance",
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value=EnvironmentVariable("USE_CAMERA", default_value="true"),
                description="Start the camera node (false for SITL, which has none)",
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
                # roslibjs can't reassemble fragments, so keep headroom above a JPEG capture.
                parameters=[{"port": 9090, "max_message_size": 10_000_000}],
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
                parameters=[{"camera_type": "arducam"}],
                condition=IfCondition(LaunchConfiguration("use_camera")),
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
                package="comms",
                executable="traffic_listener",
                name="traffic_listener",
                output="both",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="both",
                parameters=[
                    {
                        "lap_count": ParameterValue(
                            LaunchConfiguration("lap_count"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
