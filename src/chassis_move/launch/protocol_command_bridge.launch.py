"""Launch the protocol single-topic command bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the command bridge with runtime-overridable parameters."""
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
                'legacy_command_topic',
                default_value='/chassis/command_legacy',
            ),
            Node(
                package='chassis_move',
                executable='protocol_command_bridge',
                name='protocol_command_bridge',
                output='screen',
                parameters=[
                    {
                        'robot_base_url': LaunchConfiguration('robot_base_url'),
                        'robot_id': LaunchConfiguration('robot_id'),
                        'legacy_command_topic': LaunchConfiguration(
                            'legacy_command_topic'
                        ),
                    }
                ],
            )
        ]
    )
