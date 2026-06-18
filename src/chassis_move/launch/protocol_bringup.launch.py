"""Launch protocol gateway and base pose bridge services."""

# `LaunchDescription` 是 launch 文件返回的顶层对象。
from launch import LaunchDescription
# 用于查找包的 share 目录。
from ament_index_python.packages import get_package_share_directory
# `Node` 描述一个要被 launch 启动的 ROS 节点。
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch protocol callback gateway and robot status pose bridge."""
    # 找到 `chassis_move` 安装后的 share 目录。
    package_share = get_package_share_directory('chassis_move')
    # 拼出协议网关参数文件路径。
    params_file = package_share + '/config/protocol_params.yaml'

    # The protocol gateway is separated from teleop so it can be started as a
    # long-running service for any scheduler or robot callback traffic.
    # 返回最终 launch 描述。
    return LaunchDescription(
        [
            Node(
                # 协议网关所属包名。
                package='chassis_move',
                # 可执行入口名。
                executable='protocol_gateway',
                # 运行时节点名。
                name='protocol_gateway',
                # 把节点日志打印到屏幕。
                output='screen',
                # 注入参数文件。
                parameters=[params_file],
            ),
            Node(
                # 将协议状态中的 pose_msg 持续转换为 PoseStamped。
                package='chassis_move',
                executable='robot_status_pose_bridge',
                name='robot_status_pose_bridge',
                output='screen',
            ),
        ]
    )
