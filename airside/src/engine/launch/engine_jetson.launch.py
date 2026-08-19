import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mavros_distance_sensor_yaml = os.path.join(
        get_package_share_directory("engine"),
        "config",
        "mavros_distance_sensor.yaml",
    )

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
                            "distance_sensor",
                            "setpoint_raw",
                        ],
                    },
                    mavros_distance_sensor_yaml,
                ],
            ),
            Node(
                package="wrapper",
                executable="camera",
                name="camera_node",
                output="screen",
                remappings=[
                    ("camera/image_raw", "camera/image"),
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
