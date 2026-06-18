"""将统一的底盘命令消息分发到具体协议动作。"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping

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


_DEFAULT_MISSION_NAMES = {
    'MOVE': 'Move',
    'ROTATE': 'Rotate',
    'GOTO_POSE': 'GotoPose',
    'GOTO_GOAL': 'GotoGoal',
    'GO_HOME': 'GoHome',
    'CHARGE': 'Charge',
    'PATROL_GOALS': 'PatrolGoals',
}
_MISSION_COMMAND_TYPES = frozenset(_DEFAULT_MISSION_NAMES)


class CommandValidationError(ValueError):
    """命令格式或字段校验失败。"""


class MissionIdentityAllocator:
    """为桥接层自动生成可读且递增的 mission_id。"""

    def __init__(self) -> None:
        self._sequence_by_base_id: dict[str, int] = {}

    def enrich_payload(self, payload: dict[str, object]) -> dict[str, object]:
        """为 mission 型命令补齐默认 mission_name 和 mission_id。"""
        normalized = dict(payload)
        command_type = _read_required_string(normalized, 'command_type').upper()
        normalized['command_type'] = command_type

        if command_type not in _MISSION_COMMAND_TYPES:
            return normalized

        mission_name = _read_optional_string(normalized, 'mission_name')
        if mission_name is None:
            mission_name = _build_default_mission_name(command_type, normalized)
            normalized['mission_name'] = mission_name

        mission_id = _read_optional_string(normalized, 'mission_id')
        if mission_id is None:
            mission_id = self._next_mission_id(command_type, normalized, mission_name)
            normalized['mission_id'] = mission_id

        return normalized

    def _next_mission_id(
        self,
        command_type: str,
        payload: dict[str, object],
        mission_name: str,
    ) -> str:
        base_id = _build_default_mission_id_base(command_type, payload, mission_name)
        next_sequence = self._sequence_by_base_id.get(base_id, 0) + 1
        self._sequence_by_base_id[base_id] = next_sequence
        return f'{base_id}-{next_sequence:03d}'


def parse_command_payload(raw_message: str) -> dict[str, object]:
    """把 legacy `/chassis/command_legacy` 的 JSON 字符串解析为字典。"""
    try:
        payload = json.loads(raw_message)
    except json.JSONDecodeError as exc:
        raise CommandValidationError(f'invalid command json: {exc.msg}') from exc

    if not isinstance(payload, dict):
        raise CommandValidationError('command payload must be a JSON object')
    return payload


def payload_from_command_message(message: object) -> dict[str, object]:
    """把 `ChassisCommand` 转换为现有分发层使用的 payload 字典。"""
    payload = {
        'command_type': _message_string(message, 'command_type').upper(),
    }

    _maybe_set_message_string(payload, 'robot_id', message, 'robot_id')
    _maybe_set_message_string(payload, 'mission_name', message, 'mission_name')
    _maybe_set_message_string(payload, 'mission_id', message, 'mission_id')
    _maybe_set_message_string(payload, 'description', message, 'description')

    command_type = payload['command_type']

    if command_type == 'SET_MODE':
        _maybe_set_message_string(payload, 'operation_mode', message, 'operation_mode')
    elif command_type == 'MOVE':
        payload['distance_mm'] = _message_int(message, 'distance_mm')
        payload['speed_mm_s'] = _message_int(message, 'speed_mm_s')
        _maybe_set_flagged_message_int(payload, 'time_sec', message, 'has_time_sec')
    elif command_type == 'ROTATE':
        payload['angle_deg'] = _message_int(message, 'angle_deg')
        payload['speed_deg_s'] = _message_int(message, 'speed_deg_s')
        _maybe_set_flagged_message_int(
            payload,
            'angle_acc_deg_s2',
            message,
            'has_angle_acc_deg_s2',
        )
    elif command_type == 'GOTO_POSE':
        payload['x_mm'] = _message_int(message, 'x_mm')
        payload['y_mm'] = _message_int(message, 'y_mm')
        payload['heading_deg'] = _message_int(message, 'heading_deg')
        _maybe_set_flagged_message_int(payload, 'time_sec', message, 'has_time_sec')
    elif command_type == 'GOTO_GOAL':
        _maybe_set_message_string(payload, 'goal', message, 'goal')
        _maybe_set_flagged_message_int(payload, 'time_sec', message, 'has_time_sec')
    elif command_type == 'GO_HOME':
        _maybe_set_message_string(payload, 'goal', message, 'goal')
        _maybe_set_flagged_message_int(payload, 'time_sec', message, 'has_time_sec')
    elif command_type == 'CHARGE':
        _maybe_set_message_string(payload, 'goal', message, 'goal')
    elif command_type == 'PATROL_GOALS':
        _maybe_set_message_string(payload, 'from_goal', message, 'from_goal')
        _maybe_set_message_string(payload, 'to_goal', message, 'to_goal')
        _maybe_set_flagged_message_int(
            payload,
            'wait_at_goal_sec',
            message,
            'has_wait_at_goal_sec',
        )
        _maybe_set_flagged_message_int(
            payload,
            'goto_time_sec',
            message,
            'has_goto_time_sec',
        )
    elif command_type == 'REQUEST_STATUS':
        _maybe_set_message_string(payload, 'report_type', message, 'report_type')
        _maybe_set_flagged_message_int(
            payload,
            'report_duration_sec',
            message,
            'has_report_duration_sec',
        )
    elif command_type == 'CANCEL_MISSION':
        _maybe_set_message_string(payload, 'mission_type', message, 'mission_type')

    return payload


def dispatch_protocol_command(
    client: ProtocolHttpClient,
    default_robot_id: str,
    payload: dict[str, object],
) -> tuple[str, str, dict[str, object]]:
    """根据 `command_type` 调用对应的协议动作。"""
    command_type = _read_required_string(payload, 'command_type').upper()
    robot_id = _read_string(payload, 'robot_id', default_robot_id)

    if command_type == 'SET_MODE':
        response = send_set_mode(
            client=client,
            robot_id=robot_id,
            operation_mode=_read_required_string(payload, 'operation_mode'),
        )
    elif command_type == 'STOP':
        response = send_stop(client=client, robot_id=robot_id)
    elif command_type == 'MOVE':
        response = send_move_mission(
            client=client,
            robot_id=robot_id,
            distance_mm=_read_required_int(payload, 'distance_mm'),
            speed_mm_s=_read_required_int(payload, 'speed_mm_s'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['MOVE'],
            ),
            time_sec=_read_optional_int(payload, 'time_sec'),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'ROTATE':
        response = send_head_mission(
            client=client,
            robot_id=robot_id,
            angle_deg=_read_required_int(payload, 'angle_deg'),
            speed_deg_s=_read_required_int(payload, 'speed_deg_s'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['ROTATE'],
            ),
            angle_acc_deg_s2=_read_optional_int(payload, 'angle_acc_deg_s2'),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'GOTO_POSE':
        response = send_goto_pose_mission(
            client=client,
            robot_id=robot_id,
            x_mm=_read_required_int(payload, 'x_mm'),
            y_mm=_read_required_int(payload, 'y_mm'),
            heading_deg=_read_required_int(payload, 'heading_deg'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['GOTO_POSE'],
            ),
            time_sec=_read_optional_int(payload, 'time_sec'),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'GOTO_GOAL':
        response = send_goto_goal_mission(
            client=client,
            robot_id=robot_id,
            goal=_read_required_string(payload, 'goal'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['GOTO_GOAL'],
            ),
            time_sec=_read_optional_int(payload, 'time_sec'),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'GO_HOME':
        response = send_goto_goal_mission(
            client=client,
            robot_id=robot_id,
            goal=_read_string(payload, 'goal', 'Home'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['GO_HOME'],
            ),
            time_sec=_read_optional_int(payload, 'time_sec'),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'CHARGE':
        response = send_charge_mission(
            client=client,
            robot_id=robot_id,
            goal=_read_string(payload, 'goal', 'auto'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['CHARGE'],
            ),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'PATROL_GOALS':
        response = send_patrol_goals_mission(
            client=client,
            robot_id=robot_id,
            from_goal=_read_required_string(payload, 'from_goal'),
            to_goal=_read_required_string(payload, 'to_goal'),
            wait_at_goal_sec=_read_int(payload, 'wait_at_goal_sec', 10),
            goto_time_sec=_read_optional_int(payload, 'goto_time_sec'),
            mission_name=_read_string(
                payload,
                'mission_name',
                _DEFAULT_MISSION_NAMES['PATROL_GOALS'],
            ),
            mission_id=_read_optional_string(payload, 'mission_id'),
            description=_read_optional_string(payload, 'description'),
        )
    elif command_type == 'REQUEST_STATUS':
        response = send_status_request(
            client=client,
            robot_id=robot_id,
            report_type=_read_required_string(payload, 'report_type'),
            report_duration_sec=_read_optional_int(payload, 'report_duration_sec'),
        )
    elif command_type == 'CANCEL_MISSION':
        response = send_cancel_mission(
            client=client,
            robot_id=robot_id,
            mission_id=_read_required_string(payload, 'mission_id'),
            mission_name=_read_required_string(payload, 'mission_name'),
            mission_type=_read_string(payload, 'mission_type', 'normal'),
        )
    else:
        raise CommandValidationError(f'unsupported command_type: {command_type}')

    return command_type, robot_id, response


def _build_default_mission_name(
    command_type: str,
    payload: dict[str, object],
) -> str:
    if command_type == 'MOVE':
        distance_mm = _read_required_int(payload, 'distance_mm')
        if distance_mm > 0:
            return f'MoveForward{distance_mm}mm'
        if distance_mm < 0:
            return f'MoveBackward{abs(distance_mm)}mm'
        return 'Move0mm'

    if command_type == 'ROTATE':
        angle_deg = _read_required_int(payload, 'angle_deg')
        if angle_deg > 0:
            return f'RotateLeft{angle_deg}deg'
        if angle_deg < 0:
            return f'RotateRight{abs(angle_deg)}deg'
        return 'Rotate0deg'

    if command_type == 'GOTO_POSE':
        return (
            f'GotoPoseX{_read_required_int(payload, "x_mm")}'
            f'Y{_read_required_int(payload, "y_mm")}'
            f'Th{_read_required_int(payload, "heading_deg")}'
        )

    if command_type == 'GOTO_GOAL':
        return f'GotoGoal{_to_pascal_token(_read_required_string(payload, "goal"))}'

    if command_type == 'GO_HOME':
        goal = _read_string(payload, 'goal', 'Home')
        if goal.strip().lower() == 'home':
            return 'GoHome'
        return f'GotoGoal{_to_pascal_token(goal)}'

    if command_type == 'CHARGE':
        goal = _read_string(payload, 'goal', 'auto')
        return f'Charge{_to_pascal_token(goal)}'

    if command_type == 'PATROL_GOALS':
        return (
            f'Patrol'
            f'{_to_pascal_token(_read_required_string(payload, "from_goal"))}'
            f'To'
            f'{_to_pascal_token(_read_required_string(payload, "to_goal"))}'
        )

    return _DEFAULT_MISSION_NAMES[command_type]


def _build_default_mission_id_base(
    command_type: str,
    payload: dict[str, object],
    mission_name: str,
) -> str:
    if command_type == 'MOVE':
        distance_mm = _read_required_int(payload, 'distance_mm')
        if distance_mm > 0:
            return f'move-forward-{distance_mm}mm'
        if distance_mm < 0:
            return f'move-backward-{abs(distance_mm)}mm'
        return 'move-0mm'

    if command_type == 'ROTATE':
        angle_deg = _read_required_int(payload, 'angle_deg')
        if angle_deg > 0:
            return f'rotate-left-{angle_deg}deg'
        if angle_deg < 0:
            return f'rotate-right-{abs(angle_deg)}deg'
        return 'rotate-0deg'

    if command_type == 'GOTO_POSE':
        return (
            f'goto-pose-x{_read_required_int(payload, "x_mm")}'
            f'-y{_read_required_int(payload, "y_mm")}'
            f'-th{_read_required_int(payload, "heading_deg")}'
        )

    if command_type == 'GOTO_GOAL':
        return f'goto-{_to_slug_token(_read_required_string(payload, "goal"))}'

    if command_type == 'GO_HOME':
        goal = _read_string(payload, 'goal', 'Home')
        if goal.strip().lower() == 'home':
            return 'go-home'
        return f'goto-{_to_slug_token(goal)}'

    if command_type == 'CHARGE':
        goal = _read_string(payload, 'goal', 'auto')
        return f'charge-{_to_slug_token(goal)}'

    if command_type == 'PATROL_GOALS':
        return (
            f'patrol-'
            f'{_to_slug_token(_read_required_string(payload, "from_goal"))}'
            f'-to-'
            f'{_to_slug_token(_read_required_string(payload, "to_goal"))}'
        )

    return _to_slug_token(mission_name)


def _to_pascal_token(value: str) -> str:
    tokens = re.findall(r'[A-Za-z0-9]+', value)
    if not tokens:
        return 'Task'
    return ''.join(token[:1].upper() + token[1:] for token in tokens)


def _to_slug_token(value: str) -> str:
    slug = re.sub(r'[^a-z0-9]+', '-', value.strip().lower()).strip('-')
    return slug or 'task'


def _read_required_string(payload: dict[str, object], field: str) -> str:
    value = payload.get(field)
    if not isinstance(value, str) or not value.strip():
        raise CommandValidationError(f'{field} must be a non-empty string')
    return value.strip()


def _read_optional_string(payload: dict[str, object], field: str) -> str | None:
    value = payload.get(field)
    if value is None:
        return None
    if not isinstance(value, str):
        raise CommandValidationError(f'{field} must be a string')
    stripped = value.strip()
    return stripped or None


def _read_string(payload: dict[str, object], field: str, default: str) -> str:
    value = payload.get(field)
    if value is None:
        return default
    if not isinstance(value, str) or not value.strip():
        raise CommandValidationError(f'{field} must be a non-empty string')
    return value.strip()


def _read_required_int(payload: dict[str, object], field: str) -> int:
    value = payload.get(field)
    if isinstance(value, bool):
        raise CommandValidationError(f'{field} must be an integer')
    if isinstance(value, int):
        return value
    raise CommandValidationError(f'{field} must be an integer')


def _read_optional_int(payload: dict[str, object], field: str) -> int | None:
    value = payload.get(field)
    if value is None:
        return None
    if isinstance(value, bool):
        raise CommandValidationError(f'{field} must be an integer')
    if isinstance(value, int):
        return value
    raise CommandValidationError(f'{field} must be an integer')


def _read_int(payload: dict[str, object], field: str, default: int) -> int:
    value = payload.get(field)
    if value is None:
        return default
    if isinstance(value, bool):
        raise CommandValidationError(f'{field} must be an integer')
    if isinstance(value, int):
        return value
    raise CommandValidationError(f'{field} must be an integer')


def _message_value(message: object, field: str) -> object:
    if isinstance(message, Mapping):
        return message.get(field)
    return getattr(message, field, None)


def _message_string(message: object, field: str) -> str:
    value = _message_value(message, field)
    if isinstance(value, str):
        return value.strip()
    return ''


def _message_int(message: object, field: str) -> int:
    value = _message_value(message, field)
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    return 0


def _message_bool(message: object, field: str) -> bool:
    value = _message_value(message, field)
    if isinstance(value, bool):
        return value
    return False


def _maybe_set_message_string(
    payload: dict[str, object],
    payload_field: str,
    message: object,
    message_field: str,
) -> None:
    value = _message_string(message, message_field)
    if value:
        payload[payload_field] = value


def _maybe_set_flagged_message_int(
    payload: dict[str, object],
    payload_field: str,
    message: object,
    flag_field: str,
) -> None:
    if _message_bool(message, flag_field):
        payload[payload_field] = _message_int(message, payload_field)
