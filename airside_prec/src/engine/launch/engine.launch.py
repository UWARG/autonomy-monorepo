from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

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
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros",
                parameters=[
                    {"plugin_allowlist": ["sys_status","command","setpoint_position","imu","mission"]},
                    {"fcu_url": "udp://:14550@"}, #udp://:14500@ for sitl or serial:///dev/ttyUSB0:57600 for real hardware
                ],
                output="screen",
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                remappings=[
                    ("image_rect", "camera/image"),
                    ("camera_info", "camera/camera_info"),
                ],
                parameters=[os.path.join(get_package_share_directory("engine"),"apriltag.yaml")],
                output="screen",
            ),
            Node(
                package="wrapper",
                executable="mavros_comms",
                name="mavros_comms",
                output="screen",
            )
           ]
    )
