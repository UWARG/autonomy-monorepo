from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    container=ComposableNodeContainer(
        name="mavros_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::router::Router",
                name="mavros_router",
                parameters=[
                    {"fcu_urls": ["serial:///dev/serial0:57600"]},
                    {"uas_urls": ["/uas1"]},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mavros",
                plugin="mavros::uas::UAS",
                name="UAS1",
                namespace="mavros",
                parameters=[
                    {"uas_url":"/uas1"},
                    {"fcu_protocol": "v2.0"},
                    {"plugin_allowlist": ["sys_status", "command", "setpoint_position", "imu", "mission","rc_io","landing_target"]},
                    {"plugin_denylist": ["*"]},
                    {"system_id": 1},
                    {"target_system_id": 1},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "DEBUG"],
    )
    return LaunchDescription(
        [
            container,
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
           ]
    )
