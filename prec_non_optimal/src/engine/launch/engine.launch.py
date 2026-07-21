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
        parameters=[
                os.path.join(get_package_share_directory("engine"), "mavros.yaml"),
            ],
        composable_node_descriptions=[
            ComposableNode(
                package="mavros",
                plugin="mavros::router::Router",
                name="mavros_router",
                parameters=[
                    {"fcu_urls": ["serial:///dev/ttyAMA0:115200"]},
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
                    os.path.join(get_package_share_directory("engine"), "pluginlists.yaml"),
                ],
                extra_arguments=[{"use_intra_process_comms": True},
                ],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )
    return LaunchDescription(
        [
            container,
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                parameters=[
                    {"device": "/dev/video0"},
                    {"image_size": [1280, 720]},
                    {"pixel_format": "YUYV"},
                    {"output_encoding": "rgb8"},
                ],
                remappings=[
                    ("image", "camera/image"),
                ],
            ),
            Node(
                package="engine",
                executable="manager",
                name="engine_manager",
                output="screen",
            ),
            Node(
                package="nodes",
                executable="processor",
                name="processor",
                output="screen",
            ),
            Node(
                package="nodes",
                executable="controller",
                name="controller",
                output="screen",
            ),
            Node(
                package="nodes",
                executable="rc_node",
                name="rc_node",
                output="screen",
            ),
        ]
    )
