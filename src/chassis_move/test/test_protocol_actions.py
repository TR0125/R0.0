"""协议动作层的单元测试。"""

from chassis_move.protocol_actions import (
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


class FakeClient:
    """记录 post 调用参数的测试替身。"""

    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []

    def post(self, path: str, payload: dict[str, object]) -> dict[str, object]:
        self.calls.append((path, payload))
        return {'code': 0}


def test_send_patrol_goals_mission_keeps_ab_round_trip_order() -> None:
    client = FakeClient()

    send_patrol_goals_mission(
        client=client,
        robot_id='ubot-001',
        from_goal='A',
        to_goal='B',
        wait_at_goal_sec=10,
        goto_time_sec=30,
        mission_id='patrol-001',
    )

    assert len(client.calls) == 1
    path, payload = client.calls[0]
    assert path == '/mission'
    assert payload['mission_id'] == 'patrol-001'
    assert payload['robot_id'] == 'ubot-001'
    assert payload['mission_name'] == 'PatrolGoals'
    assert payload['mission_type'] == 'normal'
    tasks = payload['full_content']
    assert [task['task_id'] for task in tasks] == ['0', '1', '2']
    assert [task['cmd'] for task in tasks] == ['goto', 'wait', 'goto']
    assert tasks[0]['goal'] == 'B'
    assert tasks[2]['goal'] == 'A'


def test_send_move_mission_uses_single_task_payload() -> None:
    client = FakeClient()

    send_move_mission(
        client=client,
        robot_id='ubot-001',
        distance_mm=300,
        speed_mm_s=150,
    )

    path, payload = client.calls[0]
    assert path == '/mission'
    assert payload['full_content'][0]['task_id'] == '0'
    assert payload['full_content'][0]['cmd'] == 'move'


def test_other_actions_still_route_to_expected_protocol_paths() -> None:
    client = FakeClient()

    send_set_mode(client, 'ubot-001', 'SEMIAUTOMATIC')
    send_stop(client, 'ubot-001')
    send_status_request(client, 'ubot-001', 'path', 60)
    send_head_mission(client, 'ubot-001', 20, 40)
    send_goto_pose_mission(client, 'ubot-001', 100, 200, 90)
    send_goto_goal_mission(client, 'ubot-001', 'Home')
    send_charge_mission(client, 'ubot-001', 'auto')
    send_cancel_mission(client, 'ubot-001', 'm1', 'PatrolGoals')

    assert [path for path, _ in client.calls] == [
        '/robot/mode',
        '/command/stop',
        '/command/status',
        '/mission',
        '/mission',
        '/mission',
        '/mission',
        '/mission/cancel',
    ]
