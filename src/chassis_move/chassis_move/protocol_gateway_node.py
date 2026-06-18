"""接收机器人协议回调的 HTTP 服务节点。"""

from __future__ import annotations

# `HTTPStatus` 让状态码语义更清晰。
from http import HTTPStatus
# 标准库 HTTP Server 用于直接在节点内提供轻量服务。
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
# `json` 用于解析和转发协议消息。
import json
# HTTP 服务跑在独立线程里，避免阻塞 ROS 主循环。
import threading
# `urlparse` 用于从请求路径中提取纯路径部分。
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
# 这里用字符串消息直接承载原始 JSON 文本。
from std_msgs.msg import String


# 协议回调路径到 ROS 话题名的映射表。
_PATH_TO_TOPIC = {
    '/robot/online': '/protocol/robot_online',
    '/robot/offline': '/protocol/robot_offline',
    '/robot/login': '/protocol/robot_login',
    '/robot/logout': '/protocol/robot_logout',
    '/robot/mission/state': '/protocol/mission_state',
    '/robot/mission/task/state': '/protocol/task_state',
    '/robot/status': '/protocol/robot_status',
    '/robot/info/battery': '/protocol/robot_battery',
    '/robot/info/shape': '/protocol/robot_shape',
    '/robot/info/path': '/protocol/robot_path',
    '/robot/info/sensor': '/protocol/robot_sensor',
    '/robot/alarm': '/protocol/robot_alarm',
    '/robot/map': '/protocol/robot_map',
}


class ProtocolGatewayNode(Node):
    """在香橙派上暴露调度协议回调接口。"""

    def __init__(self) -> None:
        # 初始化 ROS 节点名。
        super().__init__('protocol_gateway')

        # 声明 HTTP 监听地址。
        self.declare_parameter('bind_host', '0.0.0.0')
        # 声明 HTTP 监听端口。
        self.declare_parameter('bind_port', 8283)
        # 声明需要返回给机器人端的服务主机地址。
        self.declare_parameter('service_host', '192.168.1.25')
        # 声明需要返回给机器人端的服务端口。
        self.declare_parameter('service_port', 8283)
        # 声明机器人上线/注册阶段可回填的注册回调地址。
        self.declare_parameter('register_endpoint', '')
        # 声明 mission 回调地址。
        self.declare_parameter('mission_endpoint', '')
        # 声明状态回调地址集合。
        self.declare_parameter('status_endpoints', '')
        # 声明命令回调地址。
        self.declare_parameter('command_endpoint', '')
        # 声明地图回调地址。
        self.declare_parameter('map_endpoint', '')

        # 将协议文档中的每个 HTTP 回调路径映射成一个 ROS topic，便于其余
        # 节点直接消费这些消息，而不需要重复实现 HTTP 服务逻辑。
        # 为每个协议回调路径创建一个字符串发布器。
        self._publishers = {
            path: self.create_publisher(String, topic, 10)
            for path, topic in _PATH_TO_TOPIC.items()
        }
        # 基于参数构建返回给机器人端的服务配置。
        self._server_config = self._build_server_config()

        # 读取监听主机地址。
        bind_host = self.get_parameter('bind_host').value
        # 读取监听端口。
        bind_port = int(self.get_parameter('bind_port').value)
        # 创建支持多线程处理请求的 HTTP 服务。
        self._server = ThreadingHTTPServer(
            (bind_host, bind_port),
            self._build_handler(),
        )
        # 让每个请求处理线程以守护线程方式运行。
        self._server.daemon_threads = True
        # 再起一个后台线程运行整个 HTTP 服务循环。
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
        )
        # 启动 HTTP 服务线程。
        self._server_thread.start()
        # 打印监听地址。
        self.get_logger().info(
            f'Protocol gateway listening on http://{bind_host}:{bind_port}'
        )
        # 打印当前会返回给机器人端的协议配置。
        self.get_logger().info(
            'Protocol server config: '
            f"{json.dumps(self._server_config, ensure_ascii=True, separators=(',', ':'))}"
        )

    def _build_handler(self) -> type[BaseHTTPRequestHandler]:
        # 把外层节点实例闭包进处理器类里，便于内部访问发布器和日志。
        node = self

        class ProtocolHandler(BaseHTTPRequestHandler):
            """处理机器人发来的 HTTP 回调请求。"""

            def do_GET(self) -> None:  # noqa: N802
                # 解析请求路径，忽略查询参数部分。
                parsed = urlparse(self.path)
                # 健康检查接口只返回一个简单成功响应。
                if parsed.path == '/health':
                    self._send_json(HTTPStatus.OK, {'code': 0, 'message': 'ok'})
                    return
                # 其他 GET 路径一律视为不存在。
                self._send_json(
                    HTTPStatus.NOT_FOUND,
                    {'code': 1002, 'message': 'not found'},
                )

            def do_POST(self) -> None:  # noqa: N802
                # 解析请求路径。
                parsed = urlparse(self.path)
                # 如果路径不在协议支持列表中，则直接返回 404。
                if parsed.path not in node._publishers:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {'code': 1002, 'message': f'unsupported path {parsed.path}'},
                    )
                    return

                # 读取请求头里的正文长度。
                content_length = int(self.headers.get('Content-Length', '0'))
                # 按长度读取请求体，并按 UTF-8 解码。
                raw_body = self.rfile.read(content_length).decode('utf-8')

                try:
                    # 有正文就解析 JSON，没有正文就按空字典处理。
                    payload = json.loads(raw_body) if raw_body else {}
                except json.JSONDecodeError:
                    # JSON 非法时返回 400。
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {'code': 1002, 'message': 'invalid json'},
                    )
                    return

                # 将解析好的回调消息发布到对应 ROS 话题。
                node._publish_payload(parsed.path, payload)
                # 按协议要求返回成功应答及必要配置。
                self._send_json(
                    HTTPStatus.OK,
                    node._build_response_payload(parsed.path),
                )

            def log_message(self, format: str, *args: object) -> None:
                # 屏蔽标准库 HTTP server 默认的 stderr 日志输出，统一交给 ROS 日志。
                return

            def _send_json(
                self,
                status: HTTPStatus,
                payload: dict[str, object],
            ) -> None:
                # 将响应字典编码成紧凑 JSON 字节串。
                body = json.dumps(
                    payload,
                    ensure_ascii=True,
                    separators=(',', ':'),
                ).encode('utf-8')
                # 写出 HTTP 状态码。
                self.send_response(status.value)
                # 标明内容类型是 JSON。
                self.send_header('Content-Type', 'application/json')
                # 写入响应体长度。
                self.send_header('Content-Length', str(len(body)))
                # 结束响应头。
                self.end_headers()
                # 写出真正的响应体。
                self.wfile.write(body)

        # 返回动态生成的处理器类给 HTTP server 使用。
        return ProtocolHandler

    def _build_server_config(self) -> dict[str, str]:
        # 读取要告诉机器人端的服务主机地址，并去掉多余空白。
        service_host = str(self.get_parameter('service_host').value).strip()
        # 读取要告诉机器人端的服务端口。
        service_port = int(self.get_parameter('service_port').value)
        # 默认状态回调地址使用 `host:port` 形式。
        default_endpoint = f'{service_host}:{service_port}'

        # 从参数中读取各类可选回调地址。
        config = {
            'register_endpoint': str(
                self.get_parameter('register_endpoint').value
            ).strip(),
            'mission_endpoint': str(
                self.get_parameter('mission_endpoint').value
            ).strip(),
            'status_endpoints': str(
                self.get_parameter('status_endpoints').value
            ).strip(),
            'command_endpoint': str(
                self.get_parameter('command_endpoint').value
            ).strip(),
            'map_endpoint': str(self.get_parameter('map_endpoint').value).strip(),
        }

        # 如果没单独配置状态回调地址，则默认填服务主机和端口。
        if not config['status_endpoints']:
            config['status_endpoints'] = default_endpoint

        # 返回整理后的配置字典。
        return config

    def _build_response_payload(self, path: str) -> dict[str, object]:
        # 所有成功响应都至少带一个 `code: 0`。
        payload: dict[str, object] = {'code': 0}

        # 协议允许在机器人 online/login 时，由服务端把当前可用回调地址返回
        # 给机器人，让机器人在本次生命周期内把状态和任务状态回推到这里。
        # 只有 online/login 这两类请求才需要附带服务端配置。
        if path in ('/robot/online', '/robot/login'):
            payload.update(
                {
                    # 只返回非空字段，避免把空字符串配置也发给机器人。
                    key: value
                    for key, value in self._server_config.items()
                    if value
                }
            )

        # 返回最终应答内容。
        return payload

    def _publish_payload(self, path: str, payload: dict[str, object]) -> None:
        # 这里保留原始 JSON 字符串，下游节点可自行决定解析方式和校验强度。
        # 创建一个 ROS 字符串消息。
        message = String()
        # 把 JSON 字典压成单行文本写入消息体。
        message.data = json.dumps(payload, ensure_ascii=True, separators=(',', ':'))
        # 发布到该路径对应的话题。
        self._publishers[path].publish(message)
        # 记录收到了一条协议回调。
        self.get_logger().info(f'Received protocol callback on {path}')

    def destroy_node(self) -> bool:
        # 主动停止 HTTP server 循环。
        self._server.shutdown()
        # 关闭监听 socket。
        self._server.server_close()
        # 如果后台线程还活着，则等待它最多 1 秒退出。
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=1.0)
        # 最后再销毁 ROS 节点本身。
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """运行协议回调网关节点。"""
    # 初始化 ROS。
    rclpy.init(args=args)
    # 创建协议网关节点。
    node = ProtocolGatewayNode()
    try:
        # 进入 ROS 事件循环。
        rclpy.spin(node)
    finally:
        # 退出时确保先释放 HTTP 资源。
        node.destroy_node()
        # 再关闭 ROS。
        rclpy.shutdown()
