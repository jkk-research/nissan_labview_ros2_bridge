from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='nissan9'),
        Node(
            package='nissan_bridge_gui',
            executable='battery_viewer',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
    ])
