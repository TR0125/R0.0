"""飞鼠手操节点的纯函数单元测试。"""

import json

from chassis_move.protocol_airmouse_node import (
    _DEFAULT_BUTTON_BINDINGS,
    compute_lateral_target_mm,
    parse_button_bindings,
    parse_pose_msg_mm,
    protocol_response_ok,
)


def test_parse_button_bindings_uses_defaults_for_invalid_json() -> None:
    bindings = parse_button_bindings('{not-json')
    assert bindings == _DEFAULT_BUTTON_BINDINGS


def test_parse_button_bindings_ignores_unsupported_actions() -> None:
    raw = json.dumps({'103': 'forward', '999': 'unknown'})
    bindings = parse_button_bindings(raw)
    assert bindings == {103: 'forward'}


def test_parse_pose_msg_mm_reads_protocol_fields() -> None:
    payload = {
        'pose_msg': {'px': 1000, 'py': 2000, 'pt': 90, 'score': 800},
        'status_msg': {'operation_mode': 'AUTOMATIC'},
    }
    pose = parse_pose_msg_mm(payload)
    assert pose == (1000.0, 2000.0, 90.0)


def test_parse_pose_msg_mm_returns_none_when_missing() -> None:
    assert parse_pose_msg_mm({'status_msg': {}}) is None


def test_compute_lateral_target_mm_moves_left_when_heading_is_east() -> None:
    target_x_mm, target_y_mm, heading_deg = compute_lateral_target_mm(
        px_mm=1000.0,
        py_mm=2000.0,
        heading_deg=90.0,
        distance_mm=300,
    )
    assert target_x_mm == 700
    assert target_y_mm == 2000
    assert heading_deg == 90


def test_protocol_response_ok_accepts_missing_code() -> None:
    assert protocol_response_ok({'message': 'ok'}) is True


def test_protocol_response_ok_rejects_non_zero_code() -> None:
    assert protocol_response_ok({'code': 1, 'message': 'busy'}) is False
