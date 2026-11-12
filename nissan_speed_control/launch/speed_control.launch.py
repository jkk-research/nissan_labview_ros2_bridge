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
            package='nissan_speed_control',
            executable='speed_control',
            namespace=namespace,
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'p_gain_accel': 0.29,
                'i_gain_accel': 0.035,
                'd_gain_accel': 0.0,
                'p_gain_brake': 0.15,
                'i_gain_brake': 0.038,
                'd_gain_brake': 0.0,
            }],
        ),
    ])
