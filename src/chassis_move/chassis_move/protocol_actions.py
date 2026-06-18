"""调度协议常用动作的共享封装。"""

from __future__ import annotations

from .protocol_client import ProtocolHttpClient
from .protocol_mission import (
    build_charge_task,
    build_full_mission,
    build_goto_goal_task,
    build_goto_pose_task,
    build_head_task,
    build_move_task,
    build_wait_task,
)


def send_set_mode(
    client: ProtocolHttpClient,
    robot_id: str,
    operation_mode: str,
) -> dict[str, object]:
    """切换机器人运行模式。"""
    return client.post(
        '/robot/mode',
        {
            'robot_id': robot_id,
            'operation_mode': operation_mode,
        },
    )


def send_stop(
    client: ProtocolHttpClient,
    robot_id: str,
) -> dict[str, object]:
    """发送急停命令。"""
    return client.post('/command/stop', {'robot_id': robot_id})


def send_status_request(
    client: ProtocolHttpClient,
    robot_id: str,
    report_type: str,
    report_duration_sec: int | None = None,
) -> dict[str, object]:
    """请求机器人主动上报某类状态。"""
    payload: dict[str, object] = {
        'robot_id': robot_id,
        'report_type': report_type,
    }
    if report_duration_sec is not None:
        payload['report_duration'] = report_duration_sec
    return client.post('/command/status', payload)


def send_move_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    distance_mm: int,
    speed_mm_s: int,
    mission_name: str = 'ManualMove',
    time_sec: int | None = None,
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送前后移动任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[build_move_task(distance_mm, speed_mm_s, time_sec)],
        description=description or f'以 {speed_mm_s} mm/s 移动 {distance_mm} mm',
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_head_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    angle_deg: int,
    speed_deg_s: int,
    mission_name: str = 'ManualHead',
    angle_acc_deg_s2: int | None = None,
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送原地旋转任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[build_head_task(angle_deg, speed_deg_s, angle_acc_deg_s2)],
        description=description or f'以 {speed_deg_s} deg/s 原地旋转 {angle_deg} 度',
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_goto_pose_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    x_mm: int,
    y_mm: int,
    heading_deg: int,
    mission_name: str = 'ManualGotoPose',
    time_sec: int | None = None,
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送绝对位姿导航任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[build_goto_pose_task(x_mm, y_mm, heading_deg, time_sec)],
        description=description
        or f'自主导航到位姿 x={x_mm} mm, y={y_mm} mm, th={heading_deg} 度',
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_goto_goal_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    goal: str,
    mission_name: str = 'ManualGotoGoal',
    time_sec: int | None = None,
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送命名目标点导航任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[build_goto_goal_task(goal, time_sec)],
        description=description or f'自主导航到目标点 {goal}',
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_charge_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    goal: str = 'auto',
    mission_name: str = 'AutoCharge',
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送自动充电任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[build_charge_task(goal)],
        mission_type='dock',
        description=description
        or (
            '自动充电：前往最近充电桩并执行充电流程'
            if goal == 'auto'
            else f'自动充电：前往充电点 {goal} 并执行充电流程'
        ),
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_patrol_goals_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    from_goal: str,
    to_goal: str,
    wait_at_goal_sec: int = 10,
    goto_time_sec: int | None = None,
    mission_name: str = 'PatrolGoals',
    mission_id: str | None = None,
    description: str | None = None,
) -> dict[str, object]:
    """发送 A-B 往返巡逻任务。"""
    mission = build_full_mission(
        robot_id=robot_id,
        mission_name=mission_name,
        tasks=[
            build_goto_goal_task(to_goal, goto_time_sec),
            build_wait_task(wait_at_goal_sec),
            build_goto_goal_task(from_goal, goto_time_sec),
        ],
        description=description
        or (
            f'从 {from_goal} 导航到 {to_goal}，'
            f'停留 {wait_at_goal_sec} 秒，'
            f'再返回 {from_goal}'
        ),
        mission_id=mission_id,
    )
    return client.post('/mission', mission)


def send_cancel_mission(
    client: ProtocolHttpClient,
    robot_id: str,
    mission_id: str,
    mission_name: str,
    mission_type: str = 'normal',
) -> dict[str, object]:
    """发送 mission 取消控制命令。"""
    return client.post(
        '/mission/cancel',
        {
            'mission_id': mission_id,
            'robot_id': robot_id,
            'mission_name': mission_name,
            'mission_type': mission_type,
            'command': 'cancel',
        },
    )
