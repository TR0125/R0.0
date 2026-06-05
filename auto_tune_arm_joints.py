#!/usr/bin/env python3

import argparse
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from right_1_arm import ALL_ARM_JOINTS, DEFAULT_JOINT_NAME, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS


@dataclass(frozen=True)
class JointActionProfile:
    positions: list[float]
    setup: dict[str, float]


def parse_joint_selection(value: str) -> list[str]:
    if value == "all":
        return list(ALL_ARM_JOINTS)
    if value == "others":
        return [joint for joint in ALL_ARM_JOINTS if joint != DEFAULT_JOINT_NAME]

    joints = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [joint for joint in joints if joint not in ALL_ARM_JOINTS]
    if unknown:
        raise ValueError(f"Unknown arm joint(s): {', '.join(unknown)}")
    return joints


def arm_joints_for(joint_name: str) -> list[str]:
    if joint_name in LEFT_ARM_JOINTS:
        return LEFT_ARM_JOINTS
    if joint_name in RIGHT_ARM_JOINTS:
        return RIGHT_ARM_JOINTS
    raise ValueError(f"Unknown arm joint: {joint_name}")


def validate_setup_joints(joint_name: str, setup: dict[str, float]) -> None:
    if not setup:
        return

    arm_joints = arm_joints_for(joint_name)
    expected = [joint for joint in arm_joints if joint != joint_name]
    expected_set = set(expected)
    actual_set = set(setup)
    missing = [joint for joint in expected if joint not in actual_set]
    extra = sorted(actual_set - expected_set)

    if missing or extra:
        details = []
        if missing:
            details.append(f"missing setup joint(s): {', '.join(missing)}")
        if extra:
            details.append(f"unexpected setup joint(s): {', '.join(extra)}")
        raise ValueError(
            f"Setup for {joint_name} must define exactly the other 6 joints in the same arm; "
            + "; ".join(details)
        )


def parse_positions(joint_name: str, value) -> list[float]:
    if not isinstance(value, list):
        raise ValueError(f"Action config for {joint_name} must be a list of positions")
    if not value:
        raise ValueError(f"Action config for {joint_name} must contain at least one position")
    return [float(item) for item in value]


def parse_setup(joint_name: str, value) -> dict[str, float]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"Setup config for {joint_name} must be a mapping of joint name to position")
    setup = {str(setup_joint): float(position) for setup_joint, position in value.items()}
    unknown = sorted(joint for joint in setup if joint not in ALL_ARM_JOINTS)
    if unknown:
        raise ValueError(f"Setup for {joint_name} contains unknown joint(s): {', '.join(unknown)}")
    validate_setup_joints(joint_name, setup)
    return setup


def parse_action_profile(joint_name: str, value) -> JointActionProfile:
    if isinstance(value, dict):
        positions_value = value.get("positions") or value.get("actions")
        positions = parse_positions(joint_name, positions_value)
        setup = parse_setup(joint_name, value.get("setup"))
        return JointActionProfile(positions=positions, setup=setup)

    return JointActionProfile(positions=parse_positions(joint_name, value), setup={})


def format_positions(positions: list[float]) -> str:
    return ",".join(f"{position:.12g}" for position in positions)


def format_setup_joints(joint_name: str, setup: dict[str, float]) -> str:
    ordered_helpers = [joint for joint in arm_joints_for(joint_name) if joint != joint_name]
    return ",".join(f"{helper}:{setup[helper]:.12g}" for helper in ordered_helpers)


def load_action_config(path_value: str) -> dict[str, JointActionProfile]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load --action-config") from exc

    path = Path(path_value).expanduser()
    with path.open("r", encoding="utf-8") as yaml_file:
        data = yaml.safe_load(yaml_file)

    if isinstance(data, dict) and isinstance(data.get("actions"), dict):
        data = data["actions"]
    if not isinstance(data, dict):
        raise ValueError("--action-config must be a mapping of joint name to positions")

    result: dict[str, JointActionProfile] = {}
    for joint_name, value in data.items():
        joint_name = str(joint_name)
        if joint_name not in ALL_ARM_JOINTS:
            raise ValueError(f"Action config contains unknown arm joint: {joint_name}")
        result[joint_name] = parse_action_profile(joint_name, value)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run right_1_arm.py Kp auto-tuning sequentially for multiple arm joints."
    )
    parser.add_argument(
        "--joints",
        default="others",
        help="Joint selection: others, all, or comma-separated joint names. Default: others",
    )
    parser.add_argument(
        "--script",
        default=str(Path(__file__).with_name("right_1_arm.py")),
        help="Path to the single-joint tuning script. Default: right_1_arm.py next to this file",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).with_name("kp_auto_tune_results")),
        help="Directory for per-joint plots and CSV files.",
    )
    parser.add_argument(
        "--auto-tune-actions",
        default=None,
        help="Comma-separated target positions used for every selected joint.",
    )
    parser.add_argument(
        "--action-config",
        default=None,
        help="YAML mapping of joint name to target positions, e.g. actions: {left_arm_1_joint: [0.1, 0.2, 0.3]}",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python executable used to launch the single-joint script.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Continue with later joints when one joint tuning run fails.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print child commands without executing them.",
    )
    parser.add_argument(
        "extra_args",
        nargs=argparse.REMAINDER,
        help="Additional arguments after -- are forwarded to right_1_arm.py.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        joints = parse_joint_selection(args.joints)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    action_config = load_action_config(args.action_config) if args.action_config else {}
    extra_args = list(args.extra_args)
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]

    script_path = Path(args.script).expanduser()
    output_root = Path(args.output_dir).expanduser()
    failures: list[tuple[str, int]] = []

    for joint_name in joints:
        joint_dir = output_root / joint_name
        raw_csv = joint_dir / f"{joint_name}_kp_trials_raw.csv"
        summary_csv = joint_dir / f"{joint_name}_kp_trials_summary.csv"
        plot_output = joint_dir / f"{joint_name}_auto_tune.png"

        command = [
            args.python,
            str(script_path),
            "--joint-name",
            joint_name,
            "--auto-tune-kp",
            "--plot-output",
            str(plot_output),
            "--kp-raw-csv-output",
            str(raw_csv),
            "--kp-summary-csv-output",
            str(summary_csv),
        ]

        if joint_name in action_config:
            profile = action_config[joint_name]
            command.append(f"--auto-tune-actions={format_positions(profile.positions)}")
            if profile.setup:
                command.append(f"--setup-joints={format_setup_joints(joint_name, profile.setup)}")
        elif args.auto_tune_actions:
            command.append(f"--auto-tune-actions={args.auto_tune_actions}")

        command.extend(extra_args)
        print(f"[{joint_name}] command: {shlex.join(command)}")
        if args.dry_run:
            continue

        joint_dir.mkdir(parents=True, exist_ok=True)
        completed = subprocess.run(command, check=False)
        if completed.returncode != 0:
            failures.append((joint_name, completed.returncode))
            print(
                f"[{joint_name}] failed with exit code {completed.returncode}",
                file=sys.stderr,
            )
            if not args.continue_on_failure:
                break

    if failures:
        print("failed_joints:", file=sys.stderr)
        for joint_name, return_code in failures:
            print(f"  {joint_name}: exit_code={return_code}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
