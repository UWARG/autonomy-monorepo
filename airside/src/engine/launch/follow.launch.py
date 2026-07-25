"""Production follow profile: no mode change, arm, or takeoff automation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_RC_PORT = 14550


def generate_launch_description() -> LaunchDescription:
    sim_target = LaunchConfiguration("sim_target")
    oakd_target = LaunchConfiguration("oakd_target")
    foxglove = LaunchConfiguration("foxglove")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value=EnvironmentVariable(
                    "FCU_URL", default_value="serial:///dev/serial0:115200"
                ),
            ),
            DeclareLaunchArgument("sim_target", default_value="false"),
            DeclareLaunchArgument("oakd_target", default_value="false"),
            DeclareLaunchArgument("blob_path", default_value=""),
            DeclareLaunchArgument("person_label", default_value="15"),
            DeclareLaunchArgument("camera_fps", default_value="20"),
            DeclareLaunchArgument("detector_stride", default_value="1"),
            DeclareLaunchArgument("max_validated_range_m", default_value="3.0"),
            DeclareLaunchArgument("world_target", default_value="false"),
            DeclareLaunchArgument("lunge", default_value="false"),
            DeclareLaunchArgument("crossing", default_value="false"),
            DeclareLaunchArgument("sim_latency_s", default_value="0.0"),
            DeclareLaunchArgument("occlusion_after_s", default_value="-1.0"),
            DeclareLaunchArgument("occlusion_duration_s", default_value="0.0"),
            DeclareLaunchArgument("drop_detector_every_n", default_value="0"),
            DeclareLaunchArgument("timing_json", default_value=""),
            DeclareLaunchArgument("props_off_hitl", default_value="false"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("enable_channel", default_value="8"),
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
                        "gcs_url": f"udp://@127.0.0.1:{_RC_PORT}",
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
                output="both",
                parameters=[{"mavlink_url": f"udpin:127.0.0.1:{_RC_PORT}"}],
            ),
            Node(
                package="wrapper",
                executable="sim_target",
                output="both",
                condition=IfCondition(sim_target),
                parameters=[
                    {
                        "world_target": ParameterValue(
                            LaunchConfiguration("world_target"), value_type=bool
                        ),
                        "lunge": ParameterValue(
                            LaunchConfiguration("lunge"), value_type=bool
                        ),
                        "crossing": ParameterValue(
                            LaunchConfiguration("crossing"), value_type=bool
                        ),
                        "detector_stride": ParameterValue(
                            LaunchConfiguration("detector_stride"), value_type=int
                        ),
                        "sim_latency_s": ParameterValue(
                            LaunchConfiguration("sim_latency_s"), value_type=float
                        ),
                        "occlusion_after_s": ParameterValue(
                            LaunchConfiguration("occlusion_after_s"), value_type=float
                        ),
                        "occlusion_duration_s": ParameterValue(
                            LaunchConfiguration("occlusion_duration_s"),
                            value_type=float,
                        ),
                        "drop_detector_every_n": ParameterValue(
                            LaunchConfiguration("drop_detector_every_n"),
                            value_type=int,
                        ),
                        "timing_json": LaunchConfiguration("timing_json"),
                    }
                ],
            ),
            Node(
                package="wrapper",
                executable="oakd_target",
                output="both",
                condition=IfCondition(oakd_target),
                parameters=[
                    {
                        "blob_path": LaunchConfiguration("blob_path"),
                        "person_label": ParameterValue(
                            LaunchConfiguration("person_label"), value_type=int
                        ),
                        "camera_fps": ParameterValue(
                            LaunchConfiguration("camera_fps"), value_type=int
                        ),
                        "detector_stride": ParameterValue(
                            LaunchConfiguration("detector_stride"), value_type=int
                        ),
                    }
                ],
            ),
            Node(
                package="engine",
                executable="follow_manager",
                output="both",
                parameters=[
                    {
                        "props_off_hitl": ParameterValue(
                            LaunchConfiguration("props_off_hitl"), value_type=bool
                        ),
                        "enable_channel": ParameterValue(
                            LaunchConfiguration("enable_channel"), value_type=int
                        ),
                        "detector_stride": ParameterValue(
                            LaunchConfiguration("detector_stride"), value_type=int
                        ),
                        "max_validated_range_m": ParameterValue(
                            LaunchConfiguration("max_validated_range_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="wrapper",
                executable="viz",
                output="screen",
                condition=IfCondition(foxglove),
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                output="screen",
                parameters=[{"port": 8765}],
                condition=IfCondition(foxglove),
            ),
        ]
    )
