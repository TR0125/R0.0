"""基于协议任务的键盘手操节点。"""

from __future__ import annotations

# `json` 用于解析机器人状态话题里的原始 JSON。
import json
# `math` 用于把朝向角换算成横移目标点。
import math
# `select` 用于非阻塞读取键盘输入。
import select
# `sys` 提供标准输入句柄。
import sys
# `termios` 用于保存与恢复终端设置。
import termios
# 键盘监听放在线程里，避免阻塞 ROS 主循环。
import threading
# `tty` 用于把终端切换到 cbreak 模式。
import tty

import rclpy
from rclpy.node import Node
# 机器人状态回调以字符串形式发布原始 JSON。
from std_msgs.msg import String

# 协议客户端和共享动作封装。
from .protocol_actions import (
    send_goto_pose_mission,
    send_head_mission,
    send_move_mission,
    send_stop,
)
from .protocol_client import ProtocolHttpClient


# 打印在终端里的帮助文本。
_HELP_TEXT = """
协议手操
  w/s : 以前后移动任务做一步前进/后退
  a/d : 基于 goto-pose 偏移做一步左移/右移
  q/e : 以 head 任务做一步左转/右转
  space: 停止机器人
  ctrl+c: 退出
"""


class ProtocolTeleopNode(Node):
    """使用协议任务命令控制机器人。"""

    def __init__(self) -> None:
        # 初始化 ROS 节点名。
        super().__init__('protocol_teleop')

        # 声明机器人协议服务基地址。
        self.declare_parameter('robot_base_url', 'http://192.168.1.55:8283')
        # 声明目标机器人 ID。
        self.declare_parameter('robot_id', 'bot-001')
        # 声明 HTTP 请求超时时间。
        self.declare_parameter('http_timeout_sec', 1.0)
        # 声明前后移动步长。
        self.declare_parameter('move_distance_mm', 300)
        # 声明前后移动速度。
        self.declare_parameter('move_speed_mm_s', 150)
        # 声明左右横移步长。
        self.declare_parameter('lateral_distance_mm', 300)
        # 声明 `goto pose` 导航超时时间。
        self.declare_parameter('goto_timeout_sec', 20)
        # 声明旋转角度步长。
        self.declare_parameter('rotate_angle_deg', 20)
        # 声明旋转角速度。
        self.declare_parameter('rotate_speed_deg_s', 40)
        # 声明订阅机器人状态的话题名。
        self.declare_parameter('status_topic', '/protocol/robot_status')

        # 该节点不发送连续速度流，而是把按键转换成协议定义的离散任务动作。
        # 读取目标机器人 ID。
        self._robot_id = self.get_parameter('robot_id').value
        # 读取前后移动步长。
        self._move_distance_mm = int(self.get_parameter('move_distance_mm').value)
        # 读取前后移动速度。
        self._move_speed_mm_s = int(self.get_parameter('move_speed_mm_s').value)
        # 读取横移步长。
        self._lateral_distance_mm = int(
            self.get_parameter('lateral_distance_mm').value
        )
        # 读取横移时使用的导航超时。
        self._goto_timeout_sec = int(self.get_parameter('goto_timeout_sec').value)
        # 读取旋转角度步长。
        self._rotate_angle_deg = int(self.get_parameter('rotate_angle_deg').value)
        # 读取旋转角速度。
        self._rotate_speed_deg_s = int(
            self.get_parameter('rotate_speed_deg_s').value
        )
        # 创建协议 HTTP 客户端，后续所有任务都通过它发往机器人。
        self._client = ProtocolHttpClient(
            base_url=self.get_parameter('robot_base_url').value,
            timeout_sec=float(self.get_parameter('http_timeout_sec').value),
        )
        # 保存最近一次从机器人状态里收到的位姿。
        self._latest_pose: tuple[float, float, float] | None = None
        # 订阅机器人状态话题，用于支持横移时的 pose 推算。
        self._status_subscription = self.create_subscription(
            String,
            self.get_parameter('status_topic').value,
            self._on_status,
            10,
        )

        # 用事件协调键盘线程退出。
        self._stop_event = threading.Event()
        # 记录当前标准输入是否支持交互式键盘控制。
        self._stdin_is_tty = sys.stdin.isatty()
        # 启动后台键盘监听线程。
        self._keyboard_thread: threading.Thread | None = None
        if self._stdin_is_tty:
            self._keyboard_thread = threading.Thread(
                target=self._keyboard_loop,
                daemon=True,
            )
            self._keyboard_thread.start()
            # 打印手操帮助。
            self.get_logger().info(_HELP_TEXT)
        else:
            self.get_logger().error(
                'stdin is not a TTY; protocol_teleop keyboard control is disabled'
            )

    def _on_status(self, message: String) -> None:
        try:
            # 把原始 JSON 字符串解析成字典。
            payload = json.loads(message.data)
            # 从协议状态中取出位姿字段。
            pose_msg = payload['pose_msg']
            # 左右平移不是协议原生动作，这里通过“基于最近位姿生成一个新的
            # goto pose 目标点”的方式做近似实现。
            # 依次保存当前位置 x、y 和朝向角。
            self._latest_pose = (
                float(pose_msg['px']),
                float(pose_msg['py']),
                float(pose_msg['pt']),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            # 状态消息格式不符合预期时，只告警并忽略。
            self.get_logger().warning('Ignoring malformed /robot/status payload')

    def _keyboard_loop(self) -> None:
        # 取得标准输入文件描述符。
        stdin_fd = sys.stdin.fileno()
        # 保存终端原始模式。
        original_settings = termios.tcgetattr(stdin_fd)
        # 切换到 cbreak 模式，让按键即时可读。
        tty.setcbreak(stdin_fd)

        try:
            while not self._stop_event.is_set():
                # 最多等待 0.1 秒轮询一次输入。
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                # 没按键时继续轮询。
                if not readable:
                    continue

                # 每次只读取一个字符。
                key = sys.stdin.read(1)
                # 空字符直接忽略。
                if not key:
                    continue

                # `Ctrl+C` 触发线程退出。
                if key == '\x03':
                    self._stop_event.set()
                    break

                # 把具体按键交给业务逻辑处理。
                self._handle_key(key)
        finally:
            # 退出时恢复终端设置。
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_settings)

    def _handle_key(self, key: str) -> None:
        try:
            # `w` 发送一步前进任务。
            if key == 'w':
                self._send_move(self._move_distance_mm)
            # `s` 发送一步后退任务。
            elif key == 's':
                self._send_move(-self._move_distance_mm)
            # `a` 发送一步左移任务。
            elif key == 'a':
                self._send_lateral(+self._lateral_distance_mm)
            # `d` 发送一步右移任务。
            elif key == 'd':
                self._send_lateral(-self._lateral_distance_mm)
            # `q` 发送一步左转任务。
            elif key == 'q':
                self._send_head(+self._rotate_angle_deg)
            # `e` 发送一步右转任务。
            elif key == 'e':
                self._send_head(-self._rotate_angle_deg)
            # 空格直接调用协议急停接口。
            elif key == ' ':
                send_stop(self._client, self._robot_id)
                self.get_logger().info('Sent /command/stop')
        except RuntimeError as exc:
            # 网络请求或响应解析失败时，统一写错误日志。
            self.get_logger().error(str(exc))

    def _send_move(self, distance_mm: int) -> None:
        # `move` 是协议里最接近“前进一步/后退一步”的标准动作。
        # 组装一条只包含单个 `move` 子任务的 mission。
        send_move_mission(
            client=self._client,
            robot_id=self._robot_id,
            distance_mm=distance_mm,
            speed_mm_s=self._move_speed_mm_s,
            mission_name='ManualMove',
            description=(
                f'键盘手操：以 {self._move_speed_mm_s} mm/s 移动 {distance_mm} mm'
            ),
        )
        # 记录发送日志。
        self.get_logger().info(f'Sent move mission distance={distance_mm}mm')

    def _send_head(self, angle_deg: int) -> None:
        # `head` 用于原地旋转，参数包括旋转角度和角速度。
        # 组装一条旋转 mission。
        send_head_mission(
            client=self._client,
            robot_id=self._robot_id,
            angle_deg=angle_deg,
            speed_deg_s=self._rotate_speed_deg_s,
            mission_name='ManualHead',
            description=(
                f'键盘手操：以 {self._rotate_speed_deg_s} deg/s 原地旋转 '
                f'{angle_deg} 度'
            ),
        )
        # 打印发送日志。
        self.get_logger().info(f'Sent head mission angle={angle_deg}deg')

    def _send_lateral(self, distance_mm: int) -> None:
        # 若还没有拿到机器人当前位姿，则无法推算横移目标点。
        if self._latest_pose is None:
            self.get_logger().warning(
                'No /robot/status pose received yet; lateral move is unavailable'
            )
            return

        # 解包最近位姿，单位均沿用协议中的毫米和度。
        px_mm, py_mm, heading_deg = self._latest_pose
        # 先把机体坐标系下的侧向偏移量换算到地图坐标系，再转成协议可接受的
        # `goto pose` 任务。
        # 将朝向角从度转换为弧度，便于三角函数计算。
        heading_rad = math.radians(heading_deg)
        # 根据朝向推算横移后的目标 X。
        target_x_mm = int(round(px_mm - distance_mm * math.sin(heading_rad)))
        # 根据朝向推算横移后的目标 Y。
        target_y_mm = int(round(py_mm + distance_mm * math.cos(heading_rad)))
        # 组装一个新的 `goto pose` mission 来近似完成横移。
        send_goto_pose_mission(
            client=self._client,
            robot_id=self._robot_id,
            x_mm=target_x_mm,
            y_mm=target_y_mm,
            heading_deg=int(round(heading_deg)),
            mission_name='ManualLateralGoto',
            time_sec=self._goto_timeout_sec,
            description=(
                f'键盘手操：基于当前位置自主导航到位姿 '
                f'({target_x_mm}, {target_y_mm}) mm，朝向 {int(round(heading_deg))} 度'
            ),
        )
        # 打印任务目标点日志。
        self.get_logger().info(
            f'Sent lateral goto mission target=({target_x_mm},{target_y_mm})mm'
        )

    def destroy_node(self) -> bool:
        # 通知键盘线程退出。
        self._stop_event.set()
        # 若线程还在运行，则等待它最多 1 秒结束。
        if self._keyboard_thread is not None and self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=1.0)
        # 最后销毁 ROS 节点资源。
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """运行协议键盘手操节点。"""
    # 初始化 ROS。
    rclpy.init(args=args)
    # 创建节点实例。
    node = ProtocolTeleopNode()
    try:
        # 进入 ROS 事件循环。
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 键盘终止属于预期退出路径。
        pass
    finally:
        # 退出前销毁节点。
        node.destroy_node()
        # 关闭 ROS。
        rclpy.shutdown()
