"""Launch protocol gateway and airmouse teleop."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Bring up callback gateway and default airmouse control chain."""
    package_share = get_package_share_directory('chassis_move')
    params_file = package_share + '/config/protocol_params.yaml'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_base_url',
                default_value='http://192.168.1.55:8283',
            ),
            DeclareLaunchArgument(
                'robot_id',
                default_value='ubot-001',
            ),
            DeclareLaunchArgument(
                'input_event_path',
                default_value='/dev/input/event0',
            ),
            DeclareLaunchArgument(
                'status_topic',
                default_value='/protocol/robot_status',
            ),
            Node(
                package='chassis_move',
                executable='protocol_gateway',
                name='protocol_gateway',
                output='screen',
                parameters=[params_file],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('chassis_move'),
                            'launch',
                            'protocol_airmouse.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'robot_base_url': LaunchConfiguration('robot_base_url'),
                    'robot_id': LaunchConfiguration('robot_id'),
                    'input_event_path': LaunchConfiguration('input_event_path'),
                    'status_topic': LaunchConfiguration('status_topic'),
                }.items(),
            ),
        ]
    )
