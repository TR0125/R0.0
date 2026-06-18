"""已落地调度协议的命令行工具。"""

from __future__ import annotations

# `argparse` 用于定义和解析命令行参数。
import argparse
# `json` 用于把响应结果格式化打印到终端。
import json
# `os` 用于读取环境变量默认值。
import os

# 引入协议客户端和共享动作封装。
from .protocol_actions import (
    send_cancel_mission,
    send_charge_mission,
    send_goto_goal_mission,
    send_goto_pose_mission,
    send_head_mission,
    send_move_mission,
    send_patrol_goals_mission,
    send_set_mode,
    send_status_request,
    send_stop,
)
from .protocol_client import ProtocolHttpClient


_DEFAULT_BASE_URL = os.environ.get('TESTMOVE_BASE_URL')
_DEFAULT_ROBOT_ID = os.environ.get('TESTMOVE_ROBOT_ID')
_DEFAULT_TIMEOUT_SEC = float(os.environ.get('TESTMOVE_TIMEOUT_SEC', '1.0'))


def _build_parser() -> argparse.ArgumentParser:
    # 这里故意保持 CLI 很薄，只做参数解析和协议映射，方便现场直接按协议
    # 调机器人，而不需要额外写测试脚本。
    # 创建根命令解析器。
    parser = argparse.ArgumentParser(description='Robot scheduler protocol CLI')
    parser.add_argument(
        '--base-url',
        default=_DEFAULT_BASE_URL,
        required=_DEFAULT_BASE_URL is None,
        help='Robot HTTP base URL, e.g. http://192.168.1.55:8283',
    )
    parser.add_argument(
        '--robot-id',
        default=_DEFAULT_ROBOT_ID,
        required=_DEFAULT_ROBOT_ID is None,
        help='Protocol robot_id',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=_DEFAULT_TIMEOUT_SEC,
        help='HTTP timeout in seconds',
    )

    # 创建子命令分发器。
    subparsers = parser.add_subparsers(dest='command', required=True)

    # `set-mode` 对应协议里的机器人运行模式切换接口。
    mode = subparsers.add_parser('set-mode', help='POST /robot/mode')
    mode.add_argument(
        'operation_mode',
        choices=['AUTOMATIC', 'SEMIAUTOMATIC', 'MANUAL', 'SERVICE'],
    )

    # `stop` 直接触发协议急停接口。
    subparsers.add_parser('stop', help='POST /command/stop')

    # `request-status` 请求机器人主动上报某类状态。
    status_request = subparsers.add_parser(
        'request-status',
        help='POST /command/status',
    )
    status_request.add_argument(
        'report_type',
        choices=['battery', 'shape', 'path', 'device'],
    )
    status_request.add_argument('--report-duration-sec', type=int, default=None)

    # 这一组子命令会新建一条 mission，并发送到 `/mission`。
    # `move` 发送一个前后直线移动任务。
    move = subparsers.add_parser('move', help='Send a full mission with cmd=move')
    move.add_argument('--distance-mm', type=int, required=True)
    move.add_argument('--speed-mm-s', type=int, required=True)
    move.add_argument('--time-sec', type=int, default=None)
    move.add_argument('--mission-name', default='ManualMove')
    move.add_argument('--mission-id', default=None)

    # `head` 发送一个原地旋转任务。
    head = subparsers.add_parser('head', help='Send a full mission with cmd=head')
    head.add_argument('--angle-deg', type=int, required=True)
    head.add_argument('--speed-deg-s', type=int, required=True)
    head.add_argument('--angle-acc-deg-s2', type=int, default=None)
    head.add_argument('--mission-name', default='ManualHead')
    head.add_argument('--mission-id', default=None)

    # `goto-pose` 发送一个基于绝对位姿的导航任务。
    goto_pose = subparsers.add_parser(
        'goto-pose',
        help='Send a full mission with cmd=goto pose',
    )
    goto_pose.add_argument('--x-mm', type=int, required=True)
    goto_pose.add_argument('--y-mm', type=int, required=True)
    goto_pose.add_argument('--th-deg', type=int, required=True)
    goto_pose.add_argument('--time-sec', type=int, default=None)
    goto_pose.add_argument('--mission-name', default='ManualGotoPose')
    goto_pose.add_argument('--mission-id', default=None)

    # `goto-goal` 发送一个基于命名目标点的导航任务。
    goto_goal = subparsers.add_parser(
        'goto-goal',
        help='Send a full mission with cmd=goto goal',
    )
    goto_goal.add_argument('--goal', required=True)
    goto_goal.add_argument('--time-sec', type=int, default=None)
    goto_goal.add_argument('--mission-name', default='ManualGotoGoal')
    goto_goal.add_argument('--mission-id', default=None)

    # `go-home` 是 `goto-goal` 的一个语义化快捷别名。
    go_home = subparsers.add_parser(
        'go-home',
        help='Send a full mission with cmd=goto goal for home',
    )
    go_home.add_argument('--goal', default='Home')
    go_home.add_argument('--time-sec', type=int, default=None)
    go_home.add_argument('--mission-name', default='GoHome')
    go_home.add_argument('--mission-id', default=None)

    # `auto-charge` 发送一个充电任务。
    auto_charge = subparsers.add_parser(
        'auto-charge',
        help='Send a full mission with cmd=charge',
    )
    auto_charge.add_argument(
        '--goal',
        default='auto',
        help='Charging goal name; use auto for the nearest charging dock',
    )
    auto_charge.add_argument('--mission-name', default='AutoCharge')
    auto_charge.add_argument('--mission-id', default=None)

    # `patrol-goals` 会在一条 mission 里串起多个子动作。
    patrol_goals = subparsers.add_parser(
        'patrol-goals',
        aliases=['patrol-ab'],
        help='Send one mission: goto target goal , return to start goal',
    )
    patrol_goals.add_argument(
        '--from-goal',
        '--goal-a',
        dest='from_goal',
        required=True,
        help='Start/return goal name',
    )
    patrol_goals.add_argument(
        '--to-goal',
        '--goal-b',
        dest='to_goal',
        required=True,
        help='Outbound goal name',
    )
    patrol_goals.add_argument(
        '--wait-at-goal-sec',
        type=int,
        default=10,
        help='Wait time in seconds after reaching the outbound goal',
    )
    patrol_goals.add_argument('--goto-time-sec', type=int, default=None)
    patrol_goals.add_argument('--mission-name', default='PatrolGoals')
    patrol_goals.add_argument('--mission-id', default=None)

    # `cancel-mission` 是取消任务的快捷命令。
    cancel_mission = subparsers.add_parser(
        'cancel-mission',
        help='Cancel a mission',
    )
    cancel_mission.add_argument('--mission-id', required=True)
    cancel_mission.add_argument('--mission-name', required=True)
    cancel_mission.add_argument('--mission-type', default='normal')

    # 返回配置完成的根解析器。
    return parser


def main() -> None:
    """运行协议命令行工具。"""
    # 构造参数解析器。
    parser = _build_parser()
    # 解析实际命令行输入。
    args = parser.parse_args()

    # 创建协议 HTTP 客户端。
    client = ProtocolHttpClient(base_url=args.base_url, timeout_sec=args.timeout_sec)

    # 每个分支都直接对应一个协议接口或一个固定的任务拼装方式。
    if args.command == 'set-mode':
        response = send_set_mode(
            client=client,
            robot_id=args.robot_id,
            operation_mode=args.operation_mode,
        )
    elif args.command == 'stop':
        response = send_stop(client=client, robot_id=args.robot_id)
    elif args.command == 'request-status':
        response = send_status_request(
            client=client,
            robot_id=args.robot_id,
            report_type=args.report_type,
            report_duration_sec=args.report_duration_sec,
        )
    elif args.command == 'move':
        response = send_move_mission(
            client=client,
            robot_id=args.robot_id,
            distance_mm=args.distance_mm,
            speed_mm_s=args.speed_mm_s,
            mission_name=args.mission_name,
            time_sec=args.time_sec,
            mission_id=args.mission_id,
        )
    elif args.command == 'head':
        response = send_head_mission(
            client=client,
            robot_id=args.robot_id,
            angle_deg=args.angle_deg,
            speed_deg_s=args.speed_deg_s,
            mission_name=args.mission_name,
            angle_acc_deg_s2=args.angle_acc_deg_s2,
            mission_id=args.mission_id,
        )
    elif args.command == 'goto-pose':
        response = send_goto_pose_mission(
            client=client,
            robot_id=args.robot_id,
            x_mm=args.x_mm,
            y_mm=args.y_mm,
            heading_deg=args.th_deg,
            mission_name=args.mission_name,
            time_sec=args.time_sec,
            mission_id=args.mission_id,
        )
    elif args.command == 'goto-goal':
        response = send_goto_goal_mission(
            client=client,
            robot_id=args.robot_id,
            goal=args.goal,
            mission_name=args.mission_name,
            time_sec=args.time_sec,
            mission_id=args.mission_id,
        )
    elif args.command == 'go-home':
        response = send_goto_goal_mission(
            client=client,
            robot_id=args.robot_id,
            goal=args.goal,
            mission_name=args.mission_name,
            time_sec=args.time_sec,
            description=f'一键回家：自主导航到目标点 {args.goal}',
            mission_id=args.mission_id,
        )
    elif args.command == 'auto-charge':
        response = send_charge_mission(
            client=client,
            robot_id=args.robot_id,
            goal=args.goal,
            mission_name=args.mission_name,
            mission_id=args.mission_id,
        )
    elif args.command in {'patrol-goals', 'patrol-ab'}:
        response = send_patrol_goals_mission(
            client=client,
            robot_id=args.robot_id,
            from_goal=args.from_goal,
            to_goal=args.to_goal,
            wait_at_goal_sec=args.wait_at_goal_sec,
            goto_time_sec=args.goto_time_sec,
            mission_name=args.mission_name,
            mission_id=args.mission_id,
        )
    elif args.command == 'cancel-mission':
        response = send_cancel_mission(
            client=client,
            robot_id=args.robot_id,
            mission_id=args.mission_id,
            mission_name=args.mission_name,
            mission_type=args.mission_type,
        )
    else:
        # 理论上 `argparse` 已经保证命令合法，进这里说明代码分支没覆盖。
        raise RuntimeError(f'Unsupported command: {args.command}')

    # 以缩进格式把响应打印到终端，方便人工查看。
    print(json.dumps(response, ensure_ascii=True, indent=2))
