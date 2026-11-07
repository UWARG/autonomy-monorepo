"""ROS bridge for the Vite dashboard; no MAVROS, camera or mission engine."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_enabled", default_value="true",
            description="Disable when reusing a running bridge and rosapi",
        ),
        DeclareLaunchArgument("bridge_bind", default_value="127.0.0.1"),
        DeclareLaunchArgument("bridge_port", default_value="9090"),
        DeclareLaunchArgument(
            "temperature_enabled", default_value="false",
            description="Enable only on the intended companion host",
        ),
        Node(
            package="rosbridge_server", executable="rosbridge_websocket",
            name="rosbridge_websocket", output="both", respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("bridge_enabled")),
            parameters=[{
                "address": LaunchConfiguration("bridge_bind"),
                "port": ParameterValue(LaunchConfiguration("bridge_port"), value_type=int),
            }],
        ),
        Node(
            package="rosapi", executable="rosapi_node", name="rosapi",
            output="both", respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("bridge_enabled")),
        ),
        Node(
            package="dashboard_monitor", executable="temperatures", output="both",
            condition=IfCondition(LaunchConfiguration("temperature_enabled")),
        ),
    ])
