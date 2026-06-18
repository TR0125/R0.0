"""Tests for the temporary chassis master publisher."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from raybot_chassis_master.command_publisher import _build_parser
except ModuleNotFoundError as exc:  # pragma: no cover - test environment gate
    pytest.skip(str(exc), allow_module_level=True)


def test_build_move_command_preserves_negative_distance() -> None:
    parser = _build_parser()
    args = parser.parse_args(
        ['move', '--distance-mm', '-300', '--speed-mm-s', '150']
    )

    assert args.command == 'move'
    assert args.distance_mm == -300
    assert args.speed_mm_s == 150


def test_build_set_mode_command_accepts_automatic() -> None:
    parser = _build_parser()
    args = parser.parse_args(['set-mode', 'AUTOMATIC'])

    assert args.command == 'set-mode'
    assert args.operation_mode == 'AUTOMATIC'
