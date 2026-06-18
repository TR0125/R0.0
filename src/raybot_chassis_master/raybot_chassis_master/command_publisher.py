"""Publish one chassis command for local integration testing."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node

from raybot_chassis_msgs.msg import ChassisCommand


def _add_common_command_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--robot-id', default='')
    parser.add_argument('--mission-name', default='')
    parser.add_argument('--mission-id', default='')
    parser.add_argument('--description', default='')


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Publish one chassis command')
    parser.add_argument('--topic', default='/chassis/command')
    subparsers = parser.add_subparsers(dest='command', required=True)

    set_mode = subparsers.add_parser('set-mode', help='Publish a SET_MODE command')
    _add_common_command_arguments(set_mode)
    set_mode.add_argument(
        'operation_mode',
        choices=['AUTOMATIC', 'SEMIAUTOMATIC', 'MANUAL', 'SERVICE'],
    )

    stop = subparsers.add_parser('stop', help='Publish a STOP command')
    _add_common_command_arguments(stop)

    move = subparsers.add_parser('move', help='Publish a MOVE command')
    _add_common_command_arguments(move)
    move.add_argument('--distance-mm', type=int, required=True)
    move.add_argument('--speed-mm-s', type=int, required=True)
    move.add_argument('--time-sec', type=int, default=None)

    rotate = subparsers.add_parser('rotate', help='Publish a ROTATE command')
    _add_common_command_arguments(rotate)
    rotate.add_argument('--angle-deg', type=int, required=True)
    rotate.add_argument('--speed-deg-s', type=int, required=True)
    rotate.add_argument('--angle-acc-deg-s2', type=int, default=None)

    goto_goal = subparsers.add_parser(
        'goto-goal',
        help='Publish a GOTO_GOAL command',
    )
    _add_common_command_arguments(goto_goal)
    goto_goal.add_argument('--goal', required=True)
    goto_goal.add_argument('--time-sec', type=int, default=None)

    patrol_goals = subparsers.add_parser(
        'patrol-goals',
        aliases=['patrol-ab'],
        help='Publish a PATROL_GOALS command',
    )
    _add_common_command_arguments(patrol_goals)
    patrol_goals.add_argument(
        '--from-goal',
        '--goal-a',
        dest='from_goal',
        required=True,
    )
    patrol_goals.add_argument(
        '--to-goal',
        '--goal-b',
        dest='to_goal',
        required=True,
    )
    patrol_goals.add_argument('--wait-at-goal-sec', type=int, default=10)
    patrol_goals.add_argument('--goto-time-sec', type=int, default=None)

    return parser


def _build_command_message(args: argparse.Namespace) -> ChassisCommand:
    message = ChassisCommand()
    message.robot_id = args.robot_id
    message.mission_name = args.mission_name
    message.mission_id = args.mission_id
    message.description = args.description

    if args.command == 'set-mode':
        message.command_type = 'SET_MODE'
        message.operation_mode = args.operation_mode
    elif args.command == 'stop':
        message.command_type = 'STOP'
    elif args.command == 'move':
        message.command_type = 'MOVE'
        message.distance_mm = args.distance_mm
        message.speed_mm_s = args.speed_mm_s
        if args.time_sec is not None:
            message.has_time_sec = True
            message.time_sec = args.time_sec
    elif args.command == 'rotate':
        message.command_type = 'ROTATE'
        message.angle_deg = args.angle_deg
        message.speed_deg_s = args.speed_deg_s
        if args.angle_acc_deg_s2 is not None:
            message.has_angle_acc_deg_s2 = True
            message.angle_acc_deg_s2 = args.angle_acc_deg_s2
    elif args.command == 'goto-goal':
        message.command_type = 'GOTO_GOAL'
        message.goal = args.goal
        if args.time_sec is not None:
            message.has_time_sec = True
            message.time_sec = args.time_sec
    elif args.command in {'patrol-goals', 'patrol-ab'}:
        message.command_type = 'PATROL_GOALS'
        message.from_goal = args.from_goal
        message.to_goal = args.to_goal
        message.has_wait_at_goal_sec = True
        message.wait_at_goal_sec = args.wait_at_goal_sec
        if args.goto_time_sec is not None:
            message.has_goto_time_sec = True
            message.goto_time_sec = args.goto_time_sec
    else:
        raise RuntimeError(f'Unsupported command: {args.command}')

    return message


class CommandPublisherNode(Node):
    """Minimal one-shot command publisher."""

    def __init__(self, topic: str) -> None:
        super().__init__('raybot_chassis_master')
        self._publisher = self.create_publisher(ChassisCommand, topic, 10)
        self._topic = topic

    def publish_once(self, message: ChassisCommand) -> None:
        self._wait_for_subscriber(timeout_sec=2.0)
        self._publisher.publish(message)
        # Give the middleware a short window to flush the one-shot message.
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info(
            f'Published {message.command_type} on {self._topic}'
        )

    def _wait_for_subscriber(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.get_logger().warning(
            f'No subscribers matched on {self._topic}; publishing anyway'
        )


def main(args: list[str] | None = None) -> None:
    """Publish a single chassis command and exit."""
    parser = _build_parser()
    parsed = parser.parse_args(args=args)
    message = _build_command_message(parsed)

    rclpy.init(args=None)
    node = CommandPublisherNode(topic=parsed.topic)
    try:
        node.publish_once(message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
