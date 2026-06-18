"""基于 Linux input event 的飞鼠按钮手操节点。"""

from __future__ import annotations

import json
import math
import os
import select
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .protocol_actions import (
    send_goto_pose_mission,
    send_head_mission,
    send_move_mission,
    send_stop,
)
from .protocol_client import ProtocolHttpClient


# Linux input event 里的按键事件类型。
_EV_KEY = 0x01

# 常见按键码默认映射，兼容飞鼠作为键盘或鼠标设备上报的场景。
_DEFAULT_BUTTON_BINDINGS: dict[int, str] = {
    103: 'forward',        # KEY_UP
    108: 'backward',       # KEY_DOWN
    105: 'lateral_left',   # KEY_LEFT
    106: 'lateral_right',  # KEY_RIGHT
    17: 'forward',         # KEY_W
    31: 'backward',        # KEY_S
    30: 'lateral_left',    # KEY_A
    32: 'lateral_right',   # KEY_D
    16: 'rotate_left',     # KEY_Q
    18: 'rotate_right',    # KEY_E
    57: 'stop',            # KEY_SPACE
    272: 'rotate_left',    # BTN_LEFT
    273: 'rotate_right',   # BTN_RIGHT
    274: 'stop',           # BTN_MIDDLE
}

# 支持的动作集合。
_SUPPORTED_ACTIONS = {
    'forward',
    'backward',
    'lateral_left',
    'lateral_right',
    'rotate_left',
    'rotate_right',
    'stop',
}


def parse_button_bindings(raw_json: str) -> dict[int, str]:
    """解析按键码到动作的 JSON 映射。"""
    try:
        parsed = json.loads(raw_json)
    except json.JSONDecodeError:
        parsed = _DEFAULT_BUTTON_BINDINGS

    if not isinstance(parsed, dict):
        parsed = _DEFAULT_BUTTON_BINDINGS

    bindings: dict[int, str] = {}
    for raw_code, raw_action in parsed.items():
        try:
            code = int(raw_code)
        except (TypeError, ValueError):
            continue

        action = str(raw_action).strip()
        if action not in _SUPPORTED_ACTIONS:
            continue

        bindings[code] = action

    if not bindings:
        return dict(_DEFAULT_BUTTON_BINDINGS)
    return bindings


def compute_lateral_target_mm(
    px_mm: float,
    py_mm: float,
    heading_deg: float,
    distance_mm: int,
) -> tuple[int, int, int]:
    """根据当前位姿计算侧移目标点（毫米）。"""
    heading_rad = math.radians(heading_deg)
    target_x_mm = int(round(px_mm - distance_mm * math.sin(heading_rad)))
    target_y_mm = int(round(py_mm + distance_mm * math.cos(heading_rad)))
    target_heading_deg = int(round(heading_deg))
    return target_x_mm, target_y_mm, target_heading_deg


def parse_pose_msg_mm(payload: dict[str, object]) -> tuple[float, float, float] | None:
    """从 /protocol/robot_status 原始 JSON 提取 pose_msg（px/py/pt，单位 mm/deg）。"""
    try:
        pose_msg = payload['pose_msg']
        if not isinstance(pose_msg, dict):
            return None
        return (
            float(pose_msg['px']),
            float(pose_msg['py']),
            float(pose_msg['pt']),
        )
    except (KeyError, TypeError, ValueError):
        return None


def protocol_response_ok(response: dict[str, object] | None) -> bool:
    """判断协议 HTTP 响应是否成功。"""
    if response is None:
        return True
    code = response.get('code')
    return code is None or code == 0


class ProtocolAirmouseNode(Node):
    """把飞鼠按钮事件转换成协议任务动作。

    左右平移直接订阅 /protocol/robot_status，读取其中的 pose_msg；
    不依赖也不发布 /raybot/base_pose。
    """

    def __init__(self) -> None:
        super().__init__('protocol_airmouse')

        self.declare_parameter('robot_base_url', 'http://192.168.1.55:8283')
        self.declare_parameter('robot_id', 'ubot-001')
        self.declare_parameter('http_timeout_sec', 1.0)
        self.declare_parameter('move_distance_mm', 300)
        self.declare_parameter('move_speed_mm_s', 150)
        self.declare_parameter('lateral_distance_mm', 300)
        self.declare_parameter('goto_timeout_sec', 20)
        self.declare_parameter('rotate_angle_deg', 20)
        self.declare_parameter('rotate_speed_deg_s', 40)
        self.declare_parameter('status_topic', '/protocol/robot_status')
        self.declare_parameter('input_event_path', '/dev/input/event0')
        self.declare_parameter(
            'button_bindings_json',
            json.dumps(_DEFAULT_BUTTON_BINDINGS, ensure_ascii=True),
        )
        self.declare_parameter('trigger_on_repeat', False)
        self.declare_parameter('min_action_interval_sec', 0.3)
        self.declare_parameter('pose_timeout_sec', 1.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._move_distance_mm = int(self.get_parameter('move_distance_mm').value)
        self._move_speed_mm_s = int(self.get_parameter('move_speed_mm_s').value)
        self._lateral_distance_mm = int(
            self.get_parameter('lateral_distance_mm').value
        )
        self._goto_timeout_sec = int(self.get_parameter('goto_timeout_sec').value)
        self._rotate_angle_deg = int(self.get_parameter('rotate_angle_deg').value)
        self._rotate_speed_deg_s = int(
            self.get_parameter('rotate_speed_deg_s').value
        )
        self._input_event_path = str(self.get_parameter('input_event_path').value)
        self._trigger_on_repeat = bool(self.get_parameter('trigger_on_repeat').value)
        self._min_action_interval_sec = float(
            self.get_parameter('min_action_interval_sec').value
        )
        self._pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)

        self._client = ProtocolHttpClient(
            base_url=self.get_parameter('robot_base_url').value,
            timeout_sec=float(self.get_parameter('http_timeout_sec').value),
        )
        self._latest_pose: tuple[float, float, float] | None = None
        self._last_pose_monotonic: float | None = None
        self._pose_lock = threading.Lock()
        self._last_action_monotonic = 0.0
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._status_subscription = self.create_subscription(
            String,
            self._status_topic,
            self._on_status,
            10,
        )

        self._button_bindings = parse_button_bindings(
            str(self.get_parameter('button_bindings_json').value)
        )

        self._stop_event = threading.Event()
        self._event_thread = threading.Thread(
            target=self._event_loop,
            daemon=True,
        )
        self._event_thread.start()

        self.get_logger().info(
            'Airmouse control enabled: event=%s status_topic=%s '
            '(lateral uses pose_msg, not base_pose) bindings=%s'
            % (self._input_event_path, self._status_topic, self._button_bindings)
        )

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring malformed /protocol/robot_status JSON')
            return

        pose = parse_pose_msg_mm(payload)
        if pose is None:
            self.get_logger().warning(
                'Ignoring /protocol/robot_status without valid pose_msg'
            )
            return

        with self._pose_lock:
            self._latest_pose = pose
            self._last_pose_monotonic = time.monotonic()

    def _event_loop(self) -> None:
        event_struct = struct.Struct('llHHI')
        fd: int | None = None

        while not self._stop_event.is_set():
            try:
                if fd is None:
                    fd = os.open(
                        self._input_event_path,
                        os.O_RDONLY | os.O_NONBLOCK,
                    )
                    self.get_logger().info(
                        f'Opened input device: {self._input_event_path}'
                    )

                readable, _, _ = select.select([fd], [], [], 0.1)
                if not readable:
                    continue

                raw_event = os.read(fd, event_struct.size)
                if len(raw_event) != event_struct.size:
                    continue

                _, _, event_type, event_code, event_value = event_struct.unpack(
                    raw_event
                )
                if event_type != _EV_KEY:
                    continue
                if event_value == 0:
                    continue
                if event_value == 2 and not self._trigger_on_repeat:
                    continue

                action = self._button_bindings.get(event_code)
                if action is None:
                    continue
                self._handle_action(action)
            except FileNotFoundError:
                if fd is not None:
                    os.close(fd)
                    fd = None
                self.get_logger().error(
                    f'Input device not found: {self._input_event_path}'
                )
                self._stop_event.wait(1.0)
            except PermissionError:
                if fd is not None:
                    os.close(fd)
                    fd = None
                self.get_logger().error(
                    'No permission to read input device %s; '
                    'please check user group or udev rules'
                    % self._input_event_path
                )
                self._stop_event.wait(1.0)
            except OSError as exc:
                if fd is not None:
                    os.close(fd)
                    fd = None
                self.get_logger().warning(f'Input device read error: {exc}')
                self._stop_event.wait(0.5)
        if fd is not None:
            os.close(fd)

    def _should_skip_action(self, action: str) -> bool:
        if self._min_action_interval_sec <= 0:
            return False

        now = time.monotonic()
        if (now - self._last_action_monotonic) < self._min_action_interval_sec:
            self.get_logger().debug(
                'Skip airmouse action "%s" due to debounce interval' % action
            )
            return True

        self._last_action_monotonic = now
        return False

    def _log_protocol_failure(self, action: str, response: dict[str, object]) -> None:
        self.get_logger().error(
            'Airmouse action "%s" rejected by chassis: %s'
            % (action, json.dumps(response, ensure_ascii=True))
        )

    def _handle_action(self, action: str) -> None:
        if self._should_skip_action(action):
            return

        try:
            if action == 'forward':
                self._send_move(self._move_distance_mm)
            elif action == 'backward':
                self._send_move(-self._move_distance_mm)
            elif action == 'lateral_left':
                self._send_lateral(+self._lateral_distance_mm)
            elif action == 'lateral_right':
                self._send_lateral(-self._lateral_distance_mm)
            elif action == 'rotate_left':
                self._send_head(+self._rotate_angle_deg)
            elif action == 'rotate_right':
                self._send_head(-self._rotate_angle_deg)
            elif action == 'stop':
                response = send_stop(self._client, self._robot_id)
                if protocol_response_ok(response):
                    self.get_logger().info('Sent /command/stop by airmouse')
                else:
                    self._log_protocol_failure(action, response)
        except RuntimeError as exc:
            self.get_logger().error(
                'Airmouse action "%s" HTTP error: %s' % (action, exc)
            )

    def _send_move(self, distance_mm: int) -> None:
        response = send_move_mission(
            client=self._client,
            robot_id=self._robot_id,
            distance_mm=distance_mm,
            speed_mm_s=self._move_speed_mm_s,
            mission_name='AirmouseMove',
            description=(
                f'飞鼠手操：以 {self._move_speed_mm_s} mm/s 移动 {distance_mm} mm'
            ),
        )
        if protocol_response_ok(response):
            self.get_logger().info(f'Sent move mission distance={distance_mm}mm')
        else:
            self._log_protocol_failure('move', response)

    def _send_head(self, angle_deg: int) -> None:
        response = send_head_mission(
            client=self._client,
            robot_id=self._robot_id,
            angle_deg=angle_deg,
            speed_deg_s=self._rotate_speed_deg_s,
            mission_name='AirmouseHead',
            description=(
                f'飞鼠手操：以 {self._rotate_speed_deg_s} deg/s 原地旋转 '
                f'{angle_deg} 度'
            ),
        )
        if protocol_response_ok(response):
            self.get_logger().info(f'Sent head mission angle={angle_deg}deg')
        else:
            self._log_protocol_failure('head', response)

    def _send_lateral(self, distance_mm: int) -> None:
        with self._pose_lock:
            latest_pose = self._latest_pose
            last_pose_monotonic = self._last_pose_monotonic

        if latest_pose is None or last_pose_monotonic is None:
            self.get_logger().warning(
                'No pose_msg on %s yet; lateral move is unavailable'
                % self._status_topic
            )
            return

        if self._pose_timeout_sec > 0:
            elapsed_sec = time.monotonic() - last_pose_monotonic
            if elapsed_sec > self._pose_timeout_sec:
                self.get_logger().warning(
                    'pose_msg is stale (%.2fs > %.2fs); lateral move skipped'
                    % (elapsed_sec, self._pose_timeout_sec)
                )
                return

        px_mm, py_mm, heading_deg = latest_pose
        target_x_mm, target_y_mm, target_heading_deg = compute_lateral_target_mm(
            px_mm,
            py_mm,
            heading_deg,
            distance_mm,
        )
        response = send_goto_pose_mission(
            client=self._client,
            robot_id=self._robot_id,
            x_mm=target_x_mm,
            y_mm=target_y_mm,
            heading_deg=target_heading_deg,
            mission_name='AirmouseLateralGoto',
            time_sec=self._goto_timeout_sec,
            description=(
                f'飞鼠手操：导航到位姿 ({target_x_mm}, {target_y_mm}) mm，'
                f'朝向 {target_heading_deg} 度'
            ),
        )
        if protocol_response_ok(response):
            self.get_logger().info(
                'Sent lateral goto from pose_msg (%d,%d)mm -> (%d,%d)mm'
                % (int(round(px_mm)), int(round(py_mm)), target_x_mm, target_y_mm)
            )
        else:
            self._log_protocol_failure('lateral', response)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._event_thread.is_alive():
            self._event_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """运行协议飞鼠按钮手操节点。"""
    rclpy.init(args=args)
    node = ProtocolAirmouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
