from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# OAK-D stereo -> stereo_sync -> stereo_odometry / rtabmap
_STEREO_NAMESPACE = "stereo"
_RGBD_IMAGE_TOPIC = f"/{_STEREO_NAMESPACE}/rgbd_image"
# OAK-D raw IMU (imu/data_raw) -> imu_filter_madgwick -> imu/data with orientation
_IMU_TOPIC = f"/{_STEREO_NAMESPACE}/imu/data"
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
                package="camera",
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
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                namespace=_STEREO_NAMESPACE,
                output="screen",
                parameters=[
                    {
                        # OAK-D has no magnetometer; orientation comes from gyro + gravity
                        "use_mag": False,
                        "publish_tf": False,
                        "world_frame": "enu",
                    }
                ],
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
                        "wait_imu_to_init": True,
                        "Odom/ResetCountdown": "1",
                    }
                ],
                remappings=[("rgbd_image", _RGBD_IMAGE_TOPIC), ("imu", _IMU_TOPIC)],
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
                        "wait_imu_to_init": True,
                        "database_path": _RTABMAP_DATABASE_PATH,
                        # RTAB-Map parameters are strings
                        "Grid/3D": "true",
                        "Grid/CellSize": "0.1",
                    }
                ],
                remappings=[("rgbd_image", _RGBD_IMAGE_TOPIC), ("imu", _IMU_TOPIC)],
                arguments=["-d"],  # start a fresh map each launch
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
