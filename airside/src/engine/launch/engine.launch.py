from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Local UDP endpoint where MAVROS mirrors the FCU stream (gcs_url) and
# where the RC bridge listens for RC_CHANNELS.
_RC_BRIDGE_PORT = 14550
_MAVROS_GCS_URL = f"udp://@127.0.0.1:{_RC_BRIDGE_PORT}"
_RC_BRIDGE_MAVLINK_URL = f"udpin:127.0.0.1:{_RC_BRIDGE_PORT}"

# OAK-D stereo -> stereo_sync -> stereo_odometry / rtabmap
_STEREO_NAMESPACE = "stereo"
_RGBD_IMAGE_TOPIC = f"/{_STEREO_NAMESPACE}/rgbd_image"
_RTABMAP_DATABASE_PATH = "/ros_ws/data/rtabmap.db"


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
                executable="oakd_stereo",
                name="oakd_stereo_node",
                namespace=_STEREO_NAMESPACE,
                output="both",
            ),
            # base_link (x fwd, z up) -> camera optical frame (z fwd, x right, y down).
            # The rotation is for a forward-facing camera; set x/y/z to the real mount offset.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="oak_tf",
                output="screen",
                arguments=[
                    "--x", "0.1", "--y", "0", "--z", "0",
                    "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
                    "--frame-id", "base_link",
                    "--child-frame-id", "oak_left_camera_optical_frame",
                ],
            ),
            Node(
                package="rtabmap_sync",
                executable="stereo_sync",
                name="stereo_sync",
                namespace=_STEREO_NAMESPACE,
                output="screen",
                parameters=[{"approx_sync": False}],
            ),
            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                name="stereo_odometry",
                output="screen",
                parameters=[
                    {
                        "frame_id": "base_link",
                        "subscribe_rgbd": True,
                        "approx_sync": False,
                        "Odom/ResetCountdown": "1"
                    }
                ],
                remappings=[("rgbd_image", _RGBD_IMAGE_TOPIC)],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    {
                        "frame_id": "base_link",
                        "subscribe_rgbd": True,
                        "subscribe_depth": False,
                        "approx_sync": False,
                        "database_path": _RTABMAP_DATABASE_PATH,
                        # RTAB-Map parameters are strings
                        "Grid/3D": "true",
                        "Grid/CellSize": "0.1",
                    }
                ],
                remappings=[("rgbd_image", _RGBD_IMAGE_TOPIC)],
                arguments=["-d"],  # start a fresh map each launch
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
