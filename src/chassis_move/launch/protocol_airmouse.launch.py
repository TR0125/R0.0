"""Launch the protocol airmouse teleop node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the airmouse node with overridable parameters."""
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
                executable='protocol_airmouse',
                name='protocol_airmouse',
                output='screen',
                parameters=[
                    {
                        'robot_base_url': LaunchConfiguration('robot_base_url'),
                        'robot_id': LaunchConfiguration('robot_id'),
                        'input_event_path': LaunchConfiguration('input_event_path'),
                        'status_topic': LaunchConfiguration('status_topic'),
                    }
                ],
            ),
        ]
    )
