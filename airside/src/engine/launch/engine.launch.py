from datetime import datetime, timezone

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Local UDP endpoint where MAVROS mirrors the FCU stream (gcs_url) and
# where the RC bridge listens for RC_CHANNELS.
_RC_BRIDGE_PORT = 14550
_MAVROS_GCS_URL = f"udp://@127.0.0.1:{_RC_BRIDGE_PORT}"
_RC_BRIDGE_MAVLINK_URL = f"udpin:127.0.0.1:{_RC_BRIDGE_PORT}"

# Topics saved to the flight recording. Raw camera images are left out
# because they fill the onboard disk in minutes.
_RECORDED_TOPICS = [
    "/mavros/state",
    "/mavros/global_position/global",
    "/mavros/global_position/rel_alt",
    "/mavros/imu/data",
    "/mavros/rangefinder/rangefinder",
    "/mavros/rc/in",
    "/mavros/setpoint_raw/global",
    "/mavros/statustext/recv",
    "/mavros/statustext/send",
    "/heartbeat",
    "/capture/target_location",
    "/trigger_post_processing",
    "/commander/abort",
    "/processed_map",
    "/targets_located",
    "/rosout",
]

# Mounted to airside/data/ on the host. ros2 bag record refuses to write
# into an existing directory, so every launch gets a timestamped name.
_BAG_DIR = "/ros_ws/data/bags"
_BAG_SPLIT_SECONDS = 300


def generate_launch_description() -> LaunchDescription:
    bag_name = datetime.now(timezone.utc).strftime("airside_%Y-%m-%dT%H-%M-%SZ")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value=EnvironmentVariable(
                    "FCU_URL", default_value="serial:///dev/serial0:115200"
                ),
                description="MAVROS connection URL to ArduPilot",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "--storage", "mcap",
                    "--storage-preset-profile", "zstd_fast",
                    "--max-bag-duration", str(_BAG_SPLIT_SECONDS),
                    "--output", f"{_BAG_DIR}/{bag_name}",
                    *_RECORDED_TOPICS,
                ],
                name="bag_recorder",
                output="both",
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
                package="wrapper",
                executable="building_target_localizer",
                name="building_target_localizer",
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
