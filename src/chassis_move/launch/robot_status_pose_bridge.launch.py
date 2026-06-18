"""Launch the robot_status pose bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch robot_status -> PoseStamped bridge with overridable parameters."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'status_topic',
                default_value='/protocol/robot_status',
            ),
            DeclareLaunchArgument(
                'base_pose_topic',
                default_value='/raybot/base_pose',
            ),
            DeclareLaunchArgument(
                'parent_frame',
                default_value='world',
            ),
            DeclareLaunchArgument(
                'publish_rate_hz',
                default_value='10.0',
            ),
            DeclareLaunchArgument(
                'position_scale',
                default_value='0.001',
            ),
            DeclareLaunchArgument(
                'yaw_is_degree',
                default_value='true',
            ),
            DeclareLaunchArgument(
                'pose_timeout_sec',
                default_value='1.0',
            ),
            Node(
                package='chassis_move',
                executable='robot_status_pose_bridge',
                name='robot_status_pose_bridge',
                output='screen',
                parameters=[
                    {
                        'status_topic': LaunchConfiguration('status_topic'),
                        'base_pose_topic': LaunchConfiguration('base_pose_topic'),
                        'parent_frame': LaunchConfiguration('parent_frame'),
                        'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                        'position_scale': LaunchConfiguration('position_scale'),
                        'yaw_is_degree': LaunchConfiguration('yaw_is_degree'),
                        'pose_timeout_sec': LaunchConfiguration('pose_timeout_sec'),
                    }
                ],
            ),
        ]
    )
