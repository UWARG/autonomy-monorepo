"""Airside flight-test launch with one explicit imaginary static obstacle."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_argument(name: str, environment: str, default: str):
    return DeclareLaunchArgument(
        name,
        default_value=EnvironmentVariable(environment, default_value=default),
    )


def generate_launch_description() -> LaunchDescription:
    base_launch = Path(get_package_share_directory("airside_bringup")) / "launch"
    return LaunchDescription(
        [
            _float_argument("obstacle_east_m", "STATIC_OBSTACLE_EAST_M", "0.0"),
            _float_argument("obstacle_north_m", "STATIC_OBSTACLE_NORTH_M", "20.0"),
            _float_argument(
                "obstacle_altitude_m", "STATIC_OBSTACLE_ALTITUDE_M", "15.0"
            ),
            _float_argument(
                "horizontal_keepaway_m", "STATIC_OBSTACLE_KEEP_AWAY_M", "5.0"
            ),
            _float_argument(
                "vertical_keepaway_m", "STATIC_OBSTACLE_VERTICAL_M", "5.0"
            ),
            _float_argument(
                "horizontal_speed_mps", "FLIGHT_TEST_HORIZONTAL_SPEED_MPS", "1.0"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch / "airside.launch.py")),
                launch_arguments={
                    "obstacle_source": "traffic",
                    "horizontal_speed_mps": LaunchConfiguration(
                        "horizontal_speed_mps"
                    ),
                }.items(),
            ),
            Node(
                package="engine",
                executable="synthetic_static_traffic",
                name="synthetic_static_traffic",
                output="both",
                parameters=[
                    {
                        "east_offset_m": ParameterValue(
                            LaunchConfiguration("obstacle_east_m"), value_type=float
                        ),
                        "north_offset_m": ParameterValue(
                            LaunchConfiguration("obstacle_north_m"), value_type=float
                        ),
                        "altitude_agl_m": ParameterValue(
                            LaunchConfiguration("obstacle_altitude_m"), value_type=float
                        ),
                        "horizontal_keepaway_m": ParameterValue(
                            LaunchConfiguration("horizontal_keepaway_m"),
                            value_type=float,
                        ),
                        "vertical_keepaway_m": ParameterValue(
                            LaunchConfiguration("vertical_keepaway_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
