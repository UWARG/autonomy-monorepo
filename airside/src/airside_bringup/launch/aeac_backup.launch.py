"""Explicit live AEAC backup; excludes the normal engine/setpoint router."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value=EnvironmentVariable(
                    "FCU_URL", default_value="serial:///dev/serial0:115200"
                ),
            ),
            DeclareLaunchArgument(
                "own_aircraft_index",
                default_value=EnvironmentVariable(
                    "AEAC_OWN_AIRCRAFT_INDEX", default_value="-1"
                ),
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="mavros",
                output="both",
                parameters=[
                    {
                        "fcu_url": LaunchConfiguration("fcu_url"),
                        "fcu_protocol": "v2.0",
                        "tgt_system": 1,
                        "tgt_component": 1,
                        "plugin_denylist": ["rc_io"],
                    }
                ],
            ),
            Node(
                package="comms",
                executable="traffic_listener",
                name="traffic_listener",
                output="both",
            ),
            Node(
                package="comms",
                executable="telemetry_sender",
                name="aeac_telemetry_sender",
                output="both",
            ),
            Node(
                package="navigation",
                executable="aeac_backup_controller",
                name="position_controller",
                output="both",
                parameters=[
                    {
                        "traffic_required": True,
                        "own_aircraft_index": ParameterValue(
                            LaunchConfiguration("own_aircraft_index"), value_type=int
                        ),
                        "traffic_freshness_s": 2.5,
                        "control_rate_hz": 5.0,
                    }
                ],
            ),
        ]
    )
