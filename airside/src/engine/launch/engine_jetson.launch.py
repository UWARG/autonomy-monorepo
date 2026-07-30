from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="mavros",
                output="screen",
                parameters=[
                    {
                        "fcu_url": "serial:///dev/ttyAMA0:115200",
                        "fcu_protocol": "v2.0",
                        "tgt_system": 1,
                        "tgt_component": 1,
                        "plugin_denylist": ["*"],
                        "plugin_allowlist": [
                            "sys_status",
                            "command",
                            "imu",
                            "mission",
                            "rc_io",
                            "global_position",
                            "setpoint_raw",
                        ],
                    },
                ],
            ),
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
                    ("image_raw", "camera/image"),
                ],
            ),
            Node(
                package="engine",
                executable="manager_jetson",
                name="engine_manager",
                output="screen",
            ),
            Node(
                package="nodes",
                executable="processor",
                name="processor",
                output="screen",
                parameters=[{"feature_method": "orb"}],
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
