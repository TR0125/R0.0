"""订阅统一底盘命令 topic 并桥接到底层协议。"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from raybot_chassis_msgs.msg import ChassisCommand

from .protocol_client import ProtocolHttpClient
from .protocol_command_bridge import (
    CommandValidationError,
    MissionIdentityAllocator,
    dispatch_protocol_command,
    parse_command_payload,
    payload_from_command_message,
)


class ProtocolCommandBridgeNode(Node):
    """把 `/chassis/command` 转换为协议 HTTP 请求。"""

    def __init__(self) -> None:
        super().__init__('protocol_command_bridge')

        self.declare_parameter('robot_base_url', 'http://192.168.1.55:8283')
        self.declare_parameter('robot_id', 'bot-001')
        self.declare_parameter('http_timeout_sec', 1.0)
        self.declare_parameter('command_topic', '/chassis/command')
        self.declare_parameter('legacy_command_topic', '/chassis/command_legacy')
        self.declare_parameter('result_topic', '/chassis/command_result')

        self._robot_id = str(self.get_parameter('robot_id').value).strip()
        self._client = ProtocolHttpClient(
            base_url=str(self.get_parameter('robot_base_url').value).strip(),
            timeout_sec=float(self.get_parameter('http_timeout_sec').value),
        )
        self._mission_identity_allocator = MissionIdentityAllocator()
        self._result_publisher = self.create_publisher(
            String,
            str(self.get_parameter('result_topic').value),
            10,
        )
        self.create_subscription(
            ChassisCommand,
            str(self.get_parameter('command_topic').value),
            self._on_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('legacy_command_topic').value),
            self._on_legacy_command,
            10,
        )

        self.get_logger().info(
            'Protocol command bridge started: '
            f"command_topic={self.get_parameter('command_topic').value}, "
            f"legacy_command_topic={self.get_parameter('legacy_command_topic').value}, "
            f"result_topic={self.get_parameter('result_topic').value}, "
            f'robot_id={self._robot_id}'
        )

    def _on_command(self, message: ChassisCommand) -> None:
        try:
            payload = self._mission_identity_allocator.enrich_payload(
                payload_from_command_message(message)
            )
            command_type, robot_id, response = dispatch_protocol_command(
                client=self._client,
                default_robot_id=self._robot_id,
                payload=payload,
            )
        except (CommandValidationError, RuntimeError) as exc:
            self.get_logger().error(f'Failed to handle /chassis/command: {exc}')
            self._publish_result(
                ok=False,
                request_payload=payload_from_command_message(message),
                error=str(exc),
            )
            return

        self.get_logger().info(
            f'Handled /chassis/command type={command_type} robot_id={robot_id}'
        )
        self._publish_result(
            ok=True,
            request_payload=payload,
            command_type=command_type,
            robot_id=robot_id,
            response=response,
        )

    def _on_legacy_command(self, message: String) -> None:
        try:
            payload = self._mission_identity_allocator.enrich_payload(
                parse_command_payload(message.data)
            )
            command_type, robot_id, response = dispatch_protocol_command(
                client=self._client,
                default_robot_id=self._robot_id,
                payload=payload,
            )
        except (CommandValidationError, RuntimeError) as exc:
            self.get_logger().error(
                f'Failed to handle /chassis/command_legacy: {exc}'
            )
            self._publish_result(
                ok=False,
                request_raw=message.data,
                error=str(exc),
            )
            return

        self.get_logger().info(
            f'Handled /chassis/command_legacy type={command_type} robot_id={robot_id}'
        )
        self._publish_result(
            ok=True,
            request_payload=payload,
            command_type=command_type,
            robot_id=robot_id,
            response=response,
        )

    def _publish_result(
        self,
        ok: bool,
        request_payload: dict[str, object] | None = None,
        request_raw: str | None = None,
        command_type: str | None = None,
        robot_id: str | None = None,
        response: dict[str, object] | None = None,
        error: str | None = None,
    ) -> None:
        result = {'ok': ok}
        if request_payload is not None:
            result['request'] = request_payload
        if request_raw is not None:
            result['request_raw'] = request_raw
        if command_type is not None:
            result['command_type'] = command_type
        if robot_id is not None:
            result['robot_id'] = robot_id
        if response is not None:
            result['response'] = response
        if error is not None:
            result['error'] = error

        message = String()
        message.data = json.dumps(result, ensure_ascii=True, separators=(',', ':'))
        self._result_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """运行协议命令桥接节点。"""
    rclpy.init(args=args)
    node = ProtocolCommandBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
