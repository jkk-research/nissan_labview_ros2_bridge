from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='nissan9'),
        DeclareLaunchArgument('frame_id', default_value=[namespace, '/base_link']),
        DeclareLaunchArgument(
            'send_control', default_value='true',
            description='Send the control cluster (ID 5) to the cRIO'),
        Node(
            package='nissan_bridge',
            executable='nissan_bridge_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'send_control': ParameterValue(
                    LaunchConfiguration('send_control'), value_type=bool),
            }],
        ),
    ])
