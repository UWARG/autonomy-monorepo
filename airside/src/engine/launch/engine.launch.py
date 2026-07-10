from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Assumes MAVROS is already running and publishing (started elsewhere) - this
# launch file does not bring it up itself.
IMS_SERVER_DIR = "/monorepo/ims/server"


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
            ExecuteProcess(
                cmd=["python3", "streamer.py"],
                cwd=IMS_SERVER_DIR,
                output="screen",
            ),
        ]
    )
