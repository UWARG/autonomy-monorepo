"""Isolated Gazebo SITL telemetry: no WARG engine, camera stub or flight nodes."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '73':
        raise RuntimeError('Use ROS_DOMAIN_ID=73 to isolate this monitor from the existing flight stack')
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', description='Verified WSL ArduPilot SITL TCP endpoint'),
        Node(package='mavros', executable='mavros_node', namespace='mavros',
             output='screen', parameters=[
                 str(Path(__file__).with_name('mavros-monitor.yaml')),
                 {'fcu_url': LaunchConfiguration('fcu_url')},
             ]),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             output='screen', parameters=[{
                 'port': 9091, 'address': '0.0.0.0',
                 'call_services_in_new_thread': True,
                 'default_call_service_timeout': 5.0,
             }]),
        Node(package='rosapi', executable='rosapi_node', output='screen'),
    ])
