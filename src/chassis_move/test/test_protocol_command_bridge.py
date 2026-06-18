"""统一底盘命令桥接逻辑测试。"""

from types import SimpleNamespace

from chassis_move.protocol_command_bridge import (
    CommandValidationError,
    MissionIdentityAllocator,
    dispatch_protocol_command,
    parse_command_payload,
    payload_from_command_message,
)


class FakeClient:
    """记录协议调用的测试替身。"""

    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []

    def post(self, path: str, payload: dict[str, object]) -> dict[str, object]:
        self.calls.append((path, payload))
        return {'code': 0, 'path': path}


def test_parse_command_payload_requires_json_object() -> None:
    try:
        parse_command_payload('[]')
    except CommandValidationError as exc:
        assert str(exc) == 'command payload must be a JSON object'
    else:
        raise AssertionError('expected CommandValidationError')


def test_payload_from_command_message_maps_move_fields() -> None:
    payload = payload_from_command_message(
        SimpleNamespace(
            command_type='move',
            robot_id='ubot-001',
            distance_mm=300,
            speed_mm_s=150,
            has_time_sec=True,
            time_sec=5,
            mission_name='ManualMove',
            mission_id='move-001',
            description='move',
        )
    )

    assert payload == {
        'command_type': 'MOVE',
        'robot_id': 'ubot-001',
        'mission_name': 'ManualMove',
        'mission_id': 'move-001',
        'description': 'move',
        'distance_mm': 300,
        'speed_mm_s': 150,
        'time_sec': 5,
    }


def test_payload_from_command_message_maps_patrol_fields() -> None:
    payload = payload_from_command_message(
        SimpleNamespace(
            command_type='PATROL_GOALS',
            from_goal='B',
            to_goal='A',
            has_wait_at_goal_sec=True,
            wait_at_goal_sec=10,
            has_goto_time_sec=False,
        )
    )

    assert payload == {
        'command_type': 'PATROL_GOALS',
        'from_goal': 'B',
        'to_goal': 'A',
        'wait_at_goal_sec': 10,
    }


def test_dispatch_move_command_routes_to_single_mission() -> None:
    client = FakeClient()
    allocator = MissionIdentityAllocator()
    payload = allocator.enrich_payload(
        {
            'command_type': 'MOVE',
            'distance_mm': 300,
            'speed_mm_s': 150,
        }
    )

    command_type, robot_id, response = dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload=payload,
    )

    assert command_type == 'MOVE'
    assert robot_id == 'ubot-001'
    assert response['path'] == '/mission'
    path, payload = client.calls[0]
    assert path == '/mission'
    assert payload['mission_name'] == 'MoveForward300mm'
    assert payload['mission_id'] == 'move-forward-300mm-001'
    assert payload['full_content'][0]['cmd'] == 'move'


def test_dispatch_rotate_command_accepts_custom_mission_fields() -> None:
    client = FakeClient()

    _, _, _ = dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload={
            'command_type': 'ROTATE',
            'robot_id': 'ubot-009',
            'angle_deg': -20,
            'speed_deg_s': 40,
            'angle_acc_deg_s2': 10,
            'mission_id': 'turn-001',
            'mission_name': 'RotateRight',
        },
    )

    path, payload = client.calls[0]
    assert path == '/mission'
    assert payload['robot_id'] == 'ubot-009'
    assert payload['mission_id'] == 'turn-001'
    assert payload['mission_name'] == 'RotateRight'
    assert payload['full_content'][0]['cmd'] == 'head'


def test_dispatch_goto_goal_and_go_home_use_same_protocol_path() -> None:
    client = FakeClient()
    allocator = MissionIdentityAllocator()

    dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload=allocator.enrich_payload(
            {'command_type': 'GOTO_GOAL', 'goal': 'A'}
        ),
    )
    dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload=allocator.enrich_payload({'command_type': 'GO_HOME'}),
    )

    assert [path for path, _ in client.calls] == ['/mission', '/mission']
    assert client.calls[0][1]['full_content'][0]['goal'] == 'A'
    assert client.calls[0][1]['mission_name'] == 'GotoGoalA'
    assert client.calls[0][1]['mission_id'] == 'goto-a-001'
    assert client.calls[1][1]['full_content'][0]['goal'] == 'Home'
    assert client.calls[1][1]['mission_name'] == 'GoHome'
    assert client.calls[1][1]['mission_id'] == 'go-home-001'


def test_dispatch_cancel_mission_requires_mission_identity() -> None:
    client = FakeClient()

    try:
        dispatch_protocol_command(
            client=client,
            default_robot_id='ubot-001',
            payload={'command_type': 'CANCEL_MISSION', 'mission_id': 'm-001'},
        )
    except CommandValidationError as exc:
        assert str(exc) == 'mission_name must be a non-empty string'
    else:
        raise AssertionError('expected CommandValidationError')


def test_dispatch_request_status_routes_to_command_status() -> None:
    client = FakeClient()

    _, _, response = dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload={
            'command_type': 'REQUEST_STATUS',
            'report_type': 'path',
            'report_duration_sec': 60,
        },
    )

    assert response['path'] == '/command/status'
    assert client.calls[0][0] == '/command/status'


def test_dispatch_patrol_goals_routes_to_single_patrol_mission() -> None:
    client = FakeClient()
    allocator = MissionIdentityAllocator()

    payload = allocator.enrich_payload(
        {
            'command_type': 'PATROL_GOALS',
            'from_goal': 'B',
            'to_goal': 'A',
            'wait_at_goal_sec': 10,
        }
    )

    command_type, robot_id, response = dispatch_protocol_command(
        client=client,
        default_robot_id='ubot-001',
        payload=payload,
    )

    assert command_type == 'PATROL_GOALS'
    assert robot_id == 'ubot-001'
    assert response['path'] == '/mission'
    path, payload = client.calls[0]
    assert path == '/mission'
    assert payload['mission_name'] == 'PatrolBToA'
    assert payload['mission_id'] == 'patrol-b-to-a-001'
    assert [task['cmd'] for task in payload['full_content']] == [
        'goto',
        'wait',
        'goto',
    ]
    assert payload['full_content'][0]['goal'] == 'A'
    assert payload['full_content'][2]['goal'] == 'B'


def test_allocator_uses_incrementing_ids_for_same_goal() -> None:
    allocator = MissionIdentityAllocator()

    first = allocator.enrich_payload({'command_type': 'GOTO_GOAL', 'goal': 'A'})
    second = allocator.enrich_payload({'command_type': 'GOTO_GOAL', 'goal': 'A'})

    assert first['mission_name'] == 'GotoGoalA'
    assert first['mission_id'] == 'goto-a-001'
    assert second['mission_name'] == 'GotoGoalA'
    assert second['mission_id'] == 'goto-a-002'


def test_allocator_uses_custom_name_to_generate_default_id() -> None:
    allocator = MissionIdentityAllocator()

    payload = allocator.enrich_payload(
        {
            'command_type': 'GOTO_GOAL',
            'goal': 'A',
            'mission_name': 'GotoGoalA',
        }
    )

    assert payload['mission_name'] == 'GotoGoalA'
    assert payload['mission_id'] == 'goto-a-001'
