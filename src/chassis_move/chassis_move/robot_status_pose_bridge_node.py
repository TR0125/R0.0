"""把 /protocol/robot_status 转换成 /raybot/base_pose。"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String


class RobotStatusPoseBridgeNode(Node):
    """将协议状态里的 pose_msg 映射为 PoseStamped。"""

    def __init__(self) -> None:
        super().__init__('robot_status_pose_bridge')

        self.declare_parameter('status_topic', '/protocol/robot_status')
        self.declare_parameter('base_pose_topic', '/raybot/base_pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('position_scale', 0.001)
        self.declare_parameter('yaw_is_degree', True)
        self.declare_parameter('pose_timeout_sec', 1.0)

        self._parent_frame = str(self.get_parameter('parent_frame').value)
        self._position_scale = float(self.get_parameter('position_scale').value)
        self._yaw_is_degree = bool(self.get_parameter('yaw_is_degree').value)
        self._pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self._last_pose_update_time: Time | None = None
        self._latest_pose_xyz_yaw: tuple[float, float, float, float] | None = None
        self._last_timeout_warn_time: Time | None = None

        self._publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('base_pose_topic').value),
            10,
        )
        self._subscription = self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_robot_status,
            10,
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        timer_period_sec = 1.0 / max(publish_rate_hz, 1e-6)
        self._timer = self.create_timer(timer_period_sec, self._publish_latest_pose)

        self.get_logger().info(
            'robot_status_pose_bridge started: '
            f"status_topic={self.get_parameter('status_topic').value}, "
            f"base_pose_topic={self.get_parameter('base_pose_topic').value}, "
            f'parent_frame={self._parent_frame}, '
            f'publish_rate_hz={publish_rate_hz}'
        )

    def _on_robot_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            pose_msg = payload['pose_msg']
            px_mm = float(pose_msg['px'])
            py_mm = float(pose_msg['py'])
            yaw = float(pose_msg['pt'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning(
                'Ignoring malformed /protocol/robot_status pose payload'
            )
            return

        x_m = px_mm * self._position_scale
        y_m = py_mm * self._position_scale
        yaw_rad = math.radians(yaw) if self._yaw_is_degree else yaw

        self._latest_pose_xyz_yaw = (x_m, y_m, 0.0, yaw_rad)
        self._last_pose_update_time = self.get_clock().now()

    def _publish_latest_pose(self) -> None:
        if self._latest_pose_xyz_yaw is None or self._last_pose_update_time is None:
            return

        now = self.get_clock().now()
        if self._pose_timeout_sec > 0.0:
            elapsed_sec = (now - self._last_pose_update_time).nanoseconds / 1e9
            if elapsed_sec > self._pose_timeout_sec:
                self._maybe_warn_timeout(now, elapsed_sec)
                return

        x_m, y_m, z_m, yaw_rad = self._latest_pose_xyz_yaw
        qz = math.sin(yaw_rad * 0.5)
        qw = math.cos(yaw_rad * 0.5)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self._parent_frame
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = z_m
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._publisher.publish(pose)

    def _maybe_warn_timeout(self, now: Time, elapsed_sec: float) -> None:
        if self._last_timeout_warn_time is not None:
            warn_gap_sec = (now - self._last_timeout_warn_time).nanoseconds / 1e9
            if warn_gap_sec < 2.0:
                return
        self._last_timeout_warn_time = now
        self.get_logger().warning(
            'Skip publishing /raybot/base_pose because robot_status pose is stale: '
            f'elapsed={elapsed_sec:.3f}s timeout={self._pose_timeout_sec:.3f}s'
        )


def main(args: list[str] | None = None) -> None:
    """运行 robot_status 到 PoseStamped 的桥接节点。"""
    rclpy.init(args=args)
    node = RobotStatusPoseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
