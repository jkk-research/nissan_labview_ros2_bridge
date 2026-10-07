from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='nissan9'),
        DeclareLaunchArgument('frame_id', default_value=[namespace, '/base_link']),
        Node(
            package='nissan_bridge',
            executable='nissan_bridge_node',
            namespace=namespace,
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')}],
        ),
    ])
