"""调度协议中的 mission 与 task 构造工具。"""

from __future__ import annotations

import time


def make_mission_id(prefix: str = 'mission') -> str:
    """为手动控制场景生成一个足够唯一的 mission_id。"""
    return f'{prefix}-{int(time.time() * 1000)}'


def build_full_mission(
    robot_id: str,
    mission_name: str,
    tasks: list[dict[str, object]],
    mission_type: str = 'normal',
    description: str = '',
    mission_id: str | None = None,
) -> dict[str, object]:
    """将任务列表封装成协议要求的 MissionMessage。"""
    mission_tasks = _with_sequential_task_ids(tasks)
    return {
        'mission_id': mission_id or make_mission_id(mission_name.lower()),
        'robot_id': robot_id,
        'mission_name': mission_name,
        'mission_type': mission_type,
        'content_type': 'full',
        'full_content': mission_tasks,
        'description': description,
    }


def build_move_task(
    distance_mm: int,
    speed_mm_s: int,
    timeout_sec: int | None = None,
) -> dict[str, object]:
    """构造一个 `move` 子任务。"""
    task: dict[str, object] = {
        'cmd': 'move',
        'distance': distance_mm,
        'speed': speed_mm_s,
    }
    if timeout_sec is not None:
        task['time'] = timeout_sec
    return task


def build_head_task(
    angle_deg: int,
    speed_deg_s: int,
    angle_acc_deg_s2: int | None = None,
) -> dict[str, object]:
    """构造一个 `head` 子任务。"""
    task: dict[str, object] = {
        'cmd': 'head',
        'angle': angle_deg,
        'speed': speed_deg_s,
    }
    if angle_acc_deg_s2 is not None:
        task['angle_acc'] = angle_acc_deg_s2
    return task


def build_wait_task(timeout_sec: int) -> dict[str, object]:
    """构造一个 `wait` 子任务。"""
    return {
        'cmd': 'wait',
        'time': timeout_sec,
    }


def build_goto_pose_task(
    x_mm: int,
    y_mm: int,
    heading_deg: int,
    timeout_sec: int | None = None,
) -> dict[str, object]:
    """构造一个使用绝对位姿目标的 `goto` 子任务。"""
    task: dict[str, object] = {
        'cmd': 'goto',
        'target': 'pose',
        'x': x_mm,
        'y': y_mm,
        'th': heading_deg,
    }
    if timeout_sec is not None:
        task['time'] = timeout_sec
    return task


def build_goto_goal_task(
    goal_name: str,
    timeout_sec: int | None = None,
) -> dict[str, object]:
    """构造一个使用命名导航点的 `goto` 子任务。"""
    task: dict[str, object] = {
        'cmd': 'goto',
        'target': 'goal',
        'goal': goal_name,
    }
    if timeout_sec is not None:
        task['time'] = timeout_sec
    return task


def build_charge_task(goal_name: str = 'auto') -> dict[str, object]:
    """构造一个 `charge` 子任务。"""
    return {
        'cmd': 'charge',
        'goal': goal_name,
    }


def _with_sequential_task_ids(tasks: list[dict[str, object]]) -> list[dict[str, object]]:
    """为 mission 中的 task 按顺序补齐稳定的 task_id。"""
    mission_tasks: list[dict[str, object]] = []
    for index, task in enumerate(tasks):
        mission_task = dict(task)
        mission_task['task_id'] = str(index)
        mission_tasks.append(mission_task)
    return mission_tasks
