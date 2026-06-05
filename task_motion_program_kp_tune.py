#!/usr/bin/env python3

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from right_1_arm import (
    ALL_ARM_JOINTS,
    ARM_JOINT_SPECS,
    AUTO_TUNE_MAX_STAGES,
    AUTO_TUNE_MIN_ITERATIONS,
    DEFAULT_JOINT_NAME,
    DEFAULT_KP_HARD_MAX_FACTOR,
    DEFAULT_KP_HARD_MAX_LIMIT,
    DEFAULT_KP_INDEX,
    DEFAULT_KP_RELATIVE_RANGE_RATIO,
    DEFAULT_KP_SUBINDEX,
    KP_BAYES_EI_TOLERANCE,
    KP_BAYES_INITIAL_SAMPLE_COUNT,
    KP_DUPLICATE_TOLERANCE,
    KP_INTERVAL_TOLERANCE,
    KP_MIN_ITERATIONS,
    allocate_stage_iteration_budgets,
    centered_initial_kp_bounds,
    choose_next_kp_via_bayes,
    next_stage_bounds,
    read_kp,
    sanitize_identifier,
    write_kp,
)


try:
    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from robot_controller.action import ProgramCommand
    from robot_controller.srv import ProgramEdit

    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    rclpy = None
    JointTrajectoryControllerState = None
    ActionClient = None
    ProgramCommand = None
    ProgramEdit = None

    class Node:  # type: ignore[no-redef]
        pass


ACTION_GROUPS_DIR = Path("/home/raybot/robot_motion_programs")
DEFAULT_OUTPUT_DIR = Path(__file__).with_name("task_motion_program_kp_results")
FAILURE_SCORE = 1_000_000.0


@dataclass
class JointSamples:
    times: list[float] = field(default_factory=list)
    references: list[float] = field(default_factory=list)
    feedbacks: list[float] = field(default_factory=list)
    errors: list[float] = field(default_factory=list)

    def add(self, elapsed: float, reference: float, feedback: float) -> None:
        self.times.append(elapsed)
        self.references.append(reference)
        self.feedbacks.append(feedback)
        self.errors.append(feedback - reference)


@dataclass
class JointTaskMetric:
    joint_name: str
    sample_count: int = 0
    rmse: float = math.inf
    mae: float = math.inf
    max_abs_error: float = math.inf
    tail_error_std: float = math.inf
    note: str = ""


@dataclass
class TaskTrackingSession:
    start_time: float
    samples_by_joint: dict[str, JointSamples]


@dataclass
class TaskRunResult:
    repeat_index: int
    success: bool
    message: str
    duration_sec: float
    metrics_by_joint: dict[str, JointTaskMetric]
    score: float = math.inf
    target_tracking_score: float = math.inf
    other_degradation_penalty: float = 0.0


@dataclass
class KpTaskEvaluation:
    tuned_joint: str
    kp: float
    role: str
    stage: str
    iteration: int
    runs: list[TaskRunResult]
    metrics_by_joint: dict[str, JointTaskMetric]
    score: float
    score_std: float
    target_tracking_score: float
    other_degradation_penalty: float
    success: bool
    message: str


@dataclass
class JointTuneSummary:
    joint_name: str
    status: str
    original_kp: float
    best_kp: float
    baseline_score: float
    best_score: float
    trial_count: int
    raw_csv: str
    summary_csv: str
    message: str = ""


class TaskMotionProgramClient(Node):
    def __init__(
        self,
        edit_service: str,
        action_name: str,
        left_state_topic: str,
        right_state_topic: str,
        monitor_joints: list[str],
    ) -> None:
        super().__init__(
            f"task_motion_program_kp_tune_{sanitize_identifier(str(int(time.time())))}"
        )
        self._edit_client = self.create_client(ProgramEdit, edit_service)
        self._action_client = ActionClient(self, ProgramCommand, action_name)
        self._monitor_joints = set(monitor_joints)
        self._latest_reference: dict[str, float] = {}
        self._latest_feedback: dict[str, float] = {}
        self._active_session: Optional[TaskTrackingSession] = None
        self._subscriptions = [
            self.create_subscription(
                JointTrajectoryControllerState,
                left_state_topic,
                lambda msg: self._controller_state_callback(msg),
                10,
            ),
            self.create_subscription(
                JointTrajectoryControllerState,
                right_state_topic,
                lambda msg: self._controller_state_callback(msg),
                10,
            ),
        ]

    def _controller_state_callback(self, msg) -> None:
        names = list(getattr(msg, "joint_names", []) or [])
        references = getattr(getattr(msg, "reference", None), "positions", [])
        feedbacks = getattr(getattr(msg, "feedback", None), "positions", [])
        if not names:
            return

        now = time.monotonic()
        session = self._active_session
        for index, joint_name in enumerate(names):
            if joint_name not in self._monitor_joints:
                continue
            try:
                reference = float(references[index])
                feedback = float(feedbacks[index])
            except (IndexError, TypeError, ValueError):
                continue
            self._latest_reference[joint_name] = reference
            self._latest_feedback[joint_name] = feedback
            if session is not None:
                samples = session.samples_by_joint.get(joint_name)
                if samples is not None:
                    samples.add(now - session.start_time, reference, feedback)

    def wait_for_ready(self, timeout_sec: float) -> None:
        if not self._edit_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("ProgramEdit service is not available")
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("ProgramCommand action server is not available")

    def wait_for_controller_state(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if all(
                joint in self._latest_reference and joint in self._latest_feedback
                for joint in self._monitor_joints
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [
            joint
            for joint in sorted(self._monitor_joints)
            if joint not in self._latest_reference or joint not in self._latest_feedback
        ]
        if missing:
            raise TimeoutError(
                f"Timed out waiting for controller_state for: {', '.join(missing)}"
            )

    def load_program(self, program_path: Path, timeout_sec: float) -> None:
        request = ProgramEdit.Request()
        request.command = f"load file={program_path}"
        future = self._edit_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError(f"Timed out loading motion program after {timeout_sec}s")
        response = future.result()
        if response is None:
            raise RuntimeError("ProgramEdit returned no response")
        if not bool(response.success):
            message = str(getattr(response, "message", "load failed"))
            raise RuntimeError(f"load motion program failed: {message}")

    def request_stop(self, timeout_sec: float = 2.0) -> None:
        try:
            request = ProgramEdit.Request()
            request.command = "stop"
            future = self._edit_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        except Exception:
            pass

    def start_tracking(self) -> TaskTrackingSession:
        session = TaskTrackingSession(
            start_time=time.monotonic(),
            samples_by_joint={joint: JointSamples() for joint in self._monitor_joints},
        )
        for joint_name in self._monitor_joints:
            if joint_name in self._latest_reference and joint_name in self._latest_feedback:
                session.samples_by_joint[joint_name].add(
                    0.0,
                    self._latest_reference[joint_name],
                    self._latest_feedback[joint_name],
                )
        self._active_session = session
        return session

    def stop_tracking(self) -> Optional[TaskTrackingSession]:
        session = self._active_session
        self._active_session = None
        return session

    def observe(self, window_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, window_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def play_program(
        self,
        play_command: str,
        send_timeout_sec: float,
        result_timeout_sec: float,
    ) -> tuple[bool, str]:
        goal_msg = ProgramCommand.Goal()
        goal_msg.command = play_command
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=send_timeout_sec)
        if not send_future.done():
            self.request_stop()
            raise TimeoutError(f"Timed out sending play goal after {send_timeout_sec}s")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ProgramCommand goal was not accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=result_timeout_sec,
        )
        if not result_future.done():
            self.request_stop()
            raise TimeoutError(f"Timed out waiting for play result after {result_timeout_sec}s")
        result_wrapper = result_future.result()
        if result_wrapper is None:
            raise RuntimeError("ProgramCommand returned no result")
        result = result_wrapper.result
        return bool(getattr(result, "success", False)), str(getattr(result, "message", ""))


def finite(value: float) -> bool:
    return math.isfinite(value)


def mean(values: list[float], default: float = math.inf) -> float:
    usable = [value for value in values if finite(value)]
    if not usable:
        return default
    return sum(usable) / len(usable)


def std(values: list[float]) -> float:
    usable = [value for value in values if finite(value)]
    if len(usable) < 2:
        return 0.0
    avg = sum(usable) / len(usable)
    return math.sqrt(sum((value - avg) ** 2 for value in usable) / len(usable))


def compute_joint_metric(joint_name: str, samples: Optional[JointSamples]) -> JointTaskMetric:
    if samples is None or not samples.errors:
        return JointTaskMetric(joint_name=joint_name, note="no samples")
    errors = samples.errors
    abs_errors = [abs(error) for error in errors]
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
    mae = sum(abs_errors) / len(abs_errors)
    tail_count = max(2, math.ceil(len(errors) * 0.25))
    tail = errors[-tail_count:]
    return JointTaskMetric(
        joint_name=joint_name,
        sample_count=len(errors),
        rmse=rmse,
        mae=mae,
        max_abs_error=max(abs_errors),
        tail_error_std=std(tail),
    )


def aggregate_joint_metrics(
    monitor_joints: list[str],
    runs: list[TaskRunResult],
) -> dict[str, JointTaskMetric]:
    aggregated: dict[str, JointTaskMetric] = {}
    for joint_name in monitor_joints:
        joint_metrics = [run.metrics_by_joint[joint_name] for run in runs]
        aggregated[joint_name] = JointTaskMetric(
            joint_name=joint_name,
            sample_count=sum(metric.sample_count for metric in joint_metrics),
            rmse=mean([metric.rmse for metric in joint_metrics]),
            mae=mean([metric.mae for metric in joint_metrics]),
            max_abs_error=mean([metric.max_abs_error for metric in joint_metrics]),
            tail_error_std=mean([metric.tail_error_std for metric in joint_metrics]),
            note="; ".join(metric.note for metric in joint_metrics if metric.note),
        )
    return aggregated


def score_metrics(
    tuned_joint: str,
    metrics_by_joint: dict[str, JointTaskMetric],
    baseline_metrics_by_joint: Optional[dict[str, JointTaskMetric]],
    args: argparse.Namespace,
    success: bool,
) -> tuple[float, float, float]:
    target = metrics_by_joint.get(tuned_joint)
    if target is None or not finite(target.rmse):
        target_score = args.failure_penalty
    else:
        target_score = (
            args.target_rmse_weight * target.rmse
            + args.target_mae_weight * target.mae
            + args.target_max_error_weight * target.max_abs_error
            + args.target_tail_std_weight * target.tail_error_std
        )

    other_degradation = 0.0
    if baseline_metrics_by_joint is not None:
        for joint_name, metric in metrics_by_joint.items():
            if joint_name == tuned_joint:
                continue
            baseline = baseline_metrics_by_joint.get(joint_name)
            if baseline is None:
                continue
            if finite(metric.rmse) and finite(baseline.rmse):
                other_degradation += max(0.0, metric.rmse - baseline.rmse)
            elif not finite(metric.rmse):
                other_degradation += args.failure_penalty

    score = target_score + args.other_degradation_weight * other_degradation
    if not success:
        score += args.failure_penalty
    return score, target_score, other_degradation


def run_motion_program_once(
    client: TaskMotionProgramClient,
    program_path: Path,
    play_command: str,
    monitor_joints: list[str],
    repeat_index: int,
    args: argparse.Namespace,
) -> TaskRunResult:
    start_time = time.monotonic()
    session = None
    success = False
    message = ""
    try:
        client.load_program(program_path, args.program_load_timeout)
        client.wait_for_controller_state(args.state_timeout)
        session = client.start_tracking()
        success, message = client.play_program(
            play_command,
            args.action_send_timeout,
            args.play_timeout,
        )
        client.observe(args.post_observe_window)
        if not success and not message:
            message = "ProgramCommand reported failure"
    except Exception as exc:
        message = str(exc)
        client.request_stop()
    finally:
        tracked_session = client.stop_tracking()
        if tracked_session is not None:
            session = tracked_session

    duration_sec = time.monotonic() - start_time
    metrics_by_joint = {
        joint_name: compute_joint_metric(
            joint_name,
            session.samples_by_joint.get(joint_name) if session is not None else None,
        )
        for joint_name in monitor_joints
    }
    return TaskRunResult(
        repeat_index=repeat_index,
        success=success,
        message=message,
        duration_sec=duration_sec,
        metrics_by_joint=metrics_by_joint,
    )


def evaluate_kp(
    client: TaskMotionProgramClient,
    program_path: Path,
    tuned_joint: str,
    kp: float,
    role: str,
    stage: str,
    iteration: int,
    monitor_joints: list[str],
    baseline_metrics_by_joint: Optional[dict[str, JointTaskMetric]],
    args: argparse.Namespace,
) -> KpTaskEvaluation:
    write_kp(args.kp_alias, args.kp_index, args.kp_subindex, kp)
    runs: list[TaskRunResult] = []
    for repeat_index in range(1, args.task_repeats + 1):
        print(
            f"[{tuned_joint}] {role} {stage} iter={iteration} "
            f"repeat={repeat_index}/{args.task_repeats} kp={kp:.6f}"
        )
        run = run_motion_program_once(
            client,
            program_path,
            args.play_command,
            monitor_joints,
            repeat_index,
            args,
        )
        run.score, run.target_tracking_score, run.other_degradation_penalty = score_metrics(
            tuned_joint,
            run.metrics_by_joint,
            baseline_metrics_by_joint,
            args,
            run.success,
        )
        runs.append(run)
        print(
            f"[{tuned_joint}] repeat={repeat_index} success={run.success} "
            f"score={run.score:.9f} message={run.message}"
        )
        if not run.success and not args.continue_on_execution_failure:
            break

    metrics_by_joint = aggregate_joint_metrics(monitor_joints, runs)
    scores = [run.score for run in runs]
    score = mean(scores, default=FAILURE_SCORE)
    target_scores = [run.target_tracking_score for run in runs]
    other_penalties = [run.other_degradation_penalty for run in runs]
    success = all(
        run.success
        and run.metrics_by_joint[tuned_joint].sample_count > 0
        and finite(run.metrics_by_joint[tuned_joint].rmse)
        for run in runs
    ) and finite(score)
    messages = [run.message for run in runs if run.message]
    return KpTaskEvaluation(
        tuned_joint=tuned_joint,
        kp=kp,
        role=role,
        stage=stage,
        iteration=iteration,
        runs=runs,
        metrics_by_joint=metrics_by_joint,
        score=score,
        score_std=std(scores),
        target_tracking_score=mean(target_scores, default=FAILURE_SCORE),
        other_degradation_penalty=mean(other_penalties, default=0.0),
        success=success,
        message="; ".join(messages),
    )


def parse_joint_selection(value: str, default: list[str]) -> list[str]:
    if value == "all":
        return list(ALL_ARM_JOINTS)
    if value == "others":
        return [joint for joint in ALL_ARM_JOINTS if joint != DEFAULT_JOINT_NAME]
    if value == "selected":
        return list(default)
    joints = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [joint for joint in joints if joint not in ALL_ARM_JOINTS]
    if unknown:
        raise ValueError(f"Unknown arm joint(s): {', '.join(unknown)}")
    return joints


def resolve_motion_program(path_value: str) -> Path:
    raw_path = Path(path_value).expanduser()
    candidates = [raw_path]
    if not raw_path.is_absolute():
        candidates.append(ACTION_GROUPS_DIR / raw_path)
        if raw_path.suffix == "":
            candidates.append(ACTION_GROUPS_DIR / f"{raw_path}.yaml")
    for candidate in candidates:
        if candidate.exists():
            resolved = candidate.resolve()
            if any(char.isspace() for char in str(resolved)):
                raise ValueError(
                    "motion program path contains whitespace, which action_recorder "
                    "cannot parse in load file=<path>"
                )
            return resolved
    raise FileNotFoundError(f"Motion program not found: {path_value}")


def determine_search_bounds(original_kp: float, args: argparse.Namespace) -> tuple[float, float, float]:
    hard_max = args.kp_hard_max
    if hard_max <= 0.0:
        hard_max = min(
            max(original_kp * DEFAULT_KP_HARD_MAX_FACTOR, original_kp + KP_INTERVAL_TOLERANCE),
            DEFAULT_KP_HARD_MAX_LIMIT,
        )
        print(
            f"kp_hard_max not specified, auto-derived: {hard_max:.6f} "
            f"(original_kp * {DEFAULT_KP_HARD_MAX_FACTOR}, clamped to <= {DEFAULT_KP_HARD_MAX_LIMIT})"
        )
    hard_max = min(hard_max, DEFAULT_KP_HARD_MAX_LIMIT)
    if original_kp > hard_max:
        raise RuntimeError(
            f"Current Kp {original_kp:.6f} exceeds configured hard max {hard_max:.6f}"
        )

    if args.kp_min is not None:
        low = max(0.0, args.kp_min)
    elif args.kp_range is not None:
        low = max(0.0, original_kp - args.kp_range)
    else:
        default_half_width = max(
            abs(original_kp) * DEFAULT_KP_RELATIVE_RANGE_RATIO,
            KP_INTERVAL_TOLERANCE,
        )
        low, _ = centered_initial_kp_bounds(original_kp, default_half_width, hard_max)

    if args.kp_max is not None:
        high = min(args.kp_max, hard_max)
    elif args.kp_range is not None:
        high = min(original_kp + args.kp_range, hard_max)
    else:
        default_half_width = max(
            abs(original_kp) * DEFAULT_KP_RELATIVE_RANGE_RATIO,
            KP_INTERVAL_TOLERANCE,
        )
        _, high = centered_initial_kp_bounds(original_kp, default_half_width, hard_max)

    if low >= high:
        raise RuntimeError(f"Kp search interval is empty after clamp: [{low:.6f}, {high:.6f}]")
    return low, high, hard_max


def save_joint_csvs(
    raw_csv: Path,
    summary_csv: Path,
    evaluations: list[KpTaskEvaluation],
    monitor_joints: list[str],
    baseline: Optional[KpTaskEvaluation],
) -> None:
    raw_csv.parent.mkdir(parents=True, exist_ok=True)
    summary_csv.parent.mkdir(parents=True, exist_ok=True)
    baseline_metrics = baseline.metrics_by_joint if baseline is not None else {}

    with raw_csv.open("w", encoding="utf-8", newline="") as csv_file:
        fieldnames = [
            "tuned_joint",
            "trial_index",
            "role",
            "stage",
            "iteration",
            "repeat_index",
            "kp",
            "success",
            "score",
            "target_tracking_score",
            "other_degradation_penalty",
            "duration_sec",
            "message",
            "monitored_joint",
            "sample_count",
            "rmse",
            "mae",
            "max_abs_error",
            "tail_error_std",
            "baseline_rmse",
            "rmse_degradation",
            "note",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for trial_index, evaluation in enumerate(evaluations):
            for run in evaluation.runs:
                for monitored_joint in monitor_joints:
                    metric = run.metrics_by_joint[monitored_joint]
                    baseline_metric = baseline_metrics.get(monitored_joint)
                    baseline_rmse = baseline_metric.rmse if baseline_metric is not None else math.inf
                    rmse_degradation = (
                        max(0.0, metric.rmse - baseline_rmse)
                        if finite(metric.rmse) and finite(baseline_rmse)
                        else math.inf
                    )
                    writer.writerow(
                        {
                            "tuned_joint": evaluation.tuned_joint,
                            "trial_index": trial_index,
                            "role": evaluation.role,
                            "stage": evaluation.stage,
                            "iteration": evaluation.iteration,
                            "repeat_index": run.repeat_index,
                            "kp": f"{evaluation.kp:.6f}",
                            "success": run.success,
                            "score": f"{run.score:.9f}",
                            "target_tracking_score": f"{run.target_tracking_score:.9f}",
                            "other_degradation_penalty": f"{run.other_degradation_penalty:.9f}",
                            "duration_sec": f"{run.duration_sec:.3f}",
                            "message": run.message,
                            "monitored_joint": monitored_joint,
                            "sample_count": metric.sample_count,
                            "rmse": f"{metric.rmse:.9f}",
                            "mae": f"{metric.mae:.9f}",
                            "max_abs_error": f"{metric.max_abs_error:.9f}",
                            "tail_error_std": f"{metric.tail_error_std:.9f}",
                            "baseline_rmse": f"{baseline_rmse:.9f}",
                            "rmse_degradation": f"{rmse_degradation:.9f}",
                            "note": metric.note,
                        }
                    )

    with summary_csv.open("w", encoding="utf-8", newline="") as csv_file:
        fieldnames = [
            "tuned_joint",
            "role",
            "stage",
            "iteration",
            "kp",
            "success",
            "score",
            "score_std",
            "repeat_count",
            "target_rmse",
            "target_mae",
            "target_max_abs_error",
            "target_tail_error_std",
            "target_tracking_score",
            "other_degradation_penalty",
            "message",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for evaluation in evaluations:
            target_metric = evaluation.metrics_by_joint[evaluation.tuned_joint]
            writer.writerow(
                {
                    "tuned_joint": evaluation.tuned_joint,
                    "role": evaluation.role,
                    "stage": evaluation.stage,
                    "iteration": evaluation.iteration,
                    "kp": f"{evaluation.kp:.6f}",
                    "success": evaluation.success,
                    "score": f"{evaluation.score:.9f}",
                    "score_std": f"{evaluation.score_std:.9f}",
                    "repeat_count": len(evaluation.runs),
                    "target_rmse": f"{target_metric.rmse:.9f}",
                    "target_mae": f"{target_metric.mae:.9f}",
                    "target_max_abs_error": f"{target_metric.max_abs_error:.9f}",
                    "target_tail_error_std": f"{target_metric.tail_error_std:.9f}",
                    "target_tracking_score": f"{evaluation.target_tracking_score:.9f}",
                    "other_degradation_penalty": f"{evaluation.other_degradation_penalty:.9f}",
                    "message": evaluation.message,
                }
            )


def save_all_joint_summary(output_path: Path, summaries: list[JointTuneSummary]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        fieldnames = [
            "joint_name",
            "status",
            "original_kp",
            "best_kp",
            "baseline_score",
            "best_score",
            "score_delta",
            "trial_count",
            "raw_csv",
            "summary_csv",
            "message",
        ]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            writer.writerow(
                {
                    "joint_name": summary.joint_name,
                    "status": summary.status,
                    "original_kp": f"{summary.original_kp:.6f}",
                    "best_kp": f"{summary.best_kp:.6f}",
                    "baseline_score": f"{summary.baseline_score:.9f}",
                    "best_score": f"{summary.best_score:.9f}",
                    "score_delta": f"{summary.best_score - summary.baseline_score:.9f}",
                    "trial_count": summary.trial_count,
                    "raw_csv": summary.raw_csv,
                    "summary_csv": summary.summary_csv,
                    "message": summary.message,
                }
            )


def choose_best_evaluation(
    current_best: Optional[KpTaskEvaluation],
    candidate: KpTaskEvaluation,
) -> Optional[KpTaskEvaluation]:
    if not candidate.success or not finite(candidate.score):
        return current_best
    if current_best is None:
        return candidate
    if candidate.score < current_best.score:
        return candidate
    return current_best


def tune_joint(
    client: TaskMotionProgramClient,
    program_path: Path,
    joint_name: str,
    monitor_joints: list[str],
    output_root: Path,
    args: argparse.Namespace,
) -> JointTuneSummary:
    spec = ARM_JOINT_SPECS[joint_name]
    args.kp_alias = spec.kp_alias
    raw_csv = output_root / joint_name / f"{joint_name}_task_kp_trials_raw.csv"
    summary_csv = output_root / joint_name / f"{joint_name}_task_kp_trials_summary.csv"
    evaluations: list[KpTaskEvaluation] = []
    baseline: Optional[KpTaskEvaluation] = None
    best: Optional[KpTaskEvaluation] = None
    original_kp = math.nan
    selected_kp = math.nan
    selected_success = False
    status = "failed"
    message = ""

    try:
        original_kp = read_kp(args.kp_alias, args.kp_index, args.kp_subindex)
        low, high, hard_max = determine_search_bounds(original_kp, args)
        print(
            f"[{joint_name}] kp_original={original_kp:.6f} "
            f"search_interval=[{low:.6f}, {high:.6f}] hard_max={hard_max:.6f}"
        )

        baseline = evaluate_kp(
            client,
            program_path,
            joint_name,
            original_kp,
            "baseline",
            "baseline",
            0,
            monitor_joints,
            None,
            args,
        )
        evaluations.append(baseline)
        best = choose_best_evaluation(best, baseline)
        save_joint_csvs(raw_csv, summary_csv, evaluations, monitor_joints, baseline)
        if not baseline.success and not args.continue_on_execution_failure:
            raise RuntimeError(f"baseline execution failed: {baseline.message}")

        stage_budgets = allocate_stage_iteration_budgets(args.kp_iterations)
        current_stage_low = low
        current_stage_high = high
        history_start_index = 0
        evaluated_by_key: dict[int, KpTaskEvaluation] = {round(original_kp, 6): baseline}
        candidate_count = 0

        for stage_index in range(AUTO_TUNE_MAX_STAGES):
            stage_label = f"stage#{stage_index + 1}"
            stage_budget = stage_budgets[min(stage_index, len(stage_budgets) - 1)]
            stage_history_limit = min(args.kp_iterations, history_start_index + stage_budget)
            stage_progress = False

            while candidate_count < stage_history_limit:
                midpoint = current_stage_low + (current_stage_high - current_stage_low) / 2.0
                seed_candidates = [current_stage_low, midpoint, current_stage_high]
                if KP_BAYES_INITIAL_SAMPLE_COUNT > 3:
                    seed_candidates.extend(
                        current_stage_low
                        + (current_stage_high - current_stage_low)
                        * index
                        / max(1, KP_BAYES_INITIAL_SAMPLE_COUNT - 1)
                        for index in range(KP_BAYES_INITIAL_SAMPLE_COUNT)
                    )
                seed_candidates = sorted(set(round(candidate, 6) for candidate in seed_candidates))
                for seed_kp in seed_candidates:
                    if candidate_count >= stage_history_limit:
                        break
                    seed_key = round(seed_kp, 6)
                    if seed_key in evaluated_by_key:
                        continue
                    candidate_count += 1
                    evaluation = evaluate_kp(
                        client,
                        program_path,
                        joint_name,
                        float(seed_kp),
                        "seed",
                        stage_label,
                        candidate_count,
                        monitor_joints,
                        baseline.metrics_by_joint,
                        args,
                    )
                    evaluations.append(evaluation)
                    evaluated_by_key[seed_key] = evaluation
                    best = choose_best_evaluation(best, evaluation)
                    save_joint_csvs(raw_csv, summary_csv, evaluations, monitor_joints, baseline)
                    stage_progress = True
                    if not evaluation.success and not args.continue_on_execution_failure:
                        raise RuntimeError(f"execution failed at kp={seed_kp:.6f}: {evaluation.message}")

                while candidate_count < stage_history_limit:
                    if current_stage_high - current_stage_low < KP_INTERVAL_TOLERANCE:
                        break
                    observed = sorted(
                        (
                            evaluation
                            for evaluation in evaluated_by_key.values()
                            if current_stage_low - KP_DUPLICATE_TOLERANCE
                            <= evaluation.kp
                            <= current_stage_high + KP_DUPLICATE_TOLERANCE
                            and finite(evaluation.score)
                        ),
                        key=lambda evaluation: evaluation.kp,
                    )
                    if len(observed) < 2:
                        break
                    candidate_kp, candidate_ei = choose_next_kp_via_bayes(
                        [evaluation.kp for evaluation in observed],
                        [evaluation.score for evaluation in observed],
                        current_stage_low,
                        current_stage_high,
                    )
                    candidate_key = round(candidate_kp, 6)
                    print(
                        f"[{joint_name}] {stage_label} bayes kp={candidate_kp:.6f} "
                        f"ei={candidate_ei:.6f}"
                    )
                    if (
                        candidate_ei < KP_BAYES_EI_TOLERANCE
                        and candidate_count >= history_start_index + KP_MIN_ITERATIONS
                    ):
                        break
                    if candidate_key in evaluated_by_key:
                        break
                    candidate_count += 1
                    evaluation = evaluate_kp(
                        client,
                        program_path,
                        joint_name,
                        candidate_kp,
                        "bayes",
                        stage_label,
                        candidate_count,
                        monitor_joints,
                        baseline.metrics_by_joint,
                        args,
                    )
                    evaluations.append(evaluation)
                    evaluated_by_key[candidate_key] = evaluation
                    best = choose_best_evaluation(best, evaluation)
                    save_joint_csvs(raw_csv, summary_csv, evaluations, monitor_joints, baseline)
                    stage_progress = True
                    if not evaluation.success and not args.continue_on_execution_failure:
                        raise RuntimeError(
                            f"execution failed at kp={candidate_kp:.6f}: {evaluation.message}"
                        )

                break

            if not stage_progress:
                break
            if candidate_count >= args.kp_iterations:
                break
            if stage_index >= AUTO_TUNE_MAX_STAGES - 1:
                break
            if best is None:
                break
            current_stage_low, current_stage_high = next_stage_bounds(
                best.kp,
                hard_max,
                stage_index + 1,
            )
            history_start_index = candidate_count
            print(
                f"[{joint_name}] next stage center={best.kp:.6f} "
                f"bounds=[{current_stage_low:.6f}, {current_stage_high:.6f}]"
            )

        if best is None:
            best = baseline
        selected_kp = best.kp
        selected_success = best.success
        status = "success" if selected_success else "failed"
        write_kp(args.kp_alias, args.kp_index, args.kp_subindex, selected_kp)
        print(
            f"[{joint_name}] selected_kp={selected_kp:.6f} "
            f"score={best.score:.9f} success={best.success}"
        )
        save_joint_csvs(raw_csv, summary_csv, evaluations, monitor_joints, baseline)
        return JointTuneSummary(
            joint_name=joint_name,
            status=status,
            original_kp=original_kp,
            best_kp=selected_kp,
            baseline_score=baseline.score if baseline is not None else math.inf,
            best_score=best.score,
            trial_count=len(evaluations),
            raw_csv=str(raw_csv),
            summary_csv=str(summary_csv),
            message=message,
        )
    except Exception as exc:
        message = str(exc)
        print(f"[{joint_name}] failed: {message}", file=sys.stderr)
        if evaluations:
            save_joint_csvs(raw_csv, summary_csv, evaluations, monitor_joints, baseline)
        return JointTuneSummary(
            joint_name=joint_name,
            status="failed",
            original_kp=original_kp,
            best_kp=selected_kp if finite(selected_kp) else original_kp,
            baseline_score=baseline.score if baseline is not None else math.inf,
            best_score=best.score if best is not None else math.inf,
            trial_count=len(evaluations),
            raw_csv=str(raw_csv),
            summary_csv=str(summary_csv),
            message=message,
        )
    finally:
        if finite(original_kp):
            restore_value = selected_kp if args.keep_best_kp and selected_success else original_kp
            try:
                write_kp(args.kp_alias, args.kp_index, args.kp_subindex, restore_value)
                if restore_value == original_kp:
                    print(f"[{joint_name}] kp_restored={restore_value:.6f}")
                else:
                    print(f"[{joint_name}] kp_kept={restore_value:.6f}")
            except Exception as exc:
                print(f"[{joint_name}] kp_restore_failed: {exc}", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Tune arm joint Kp values by executing a complete motion-program YAML "
            "for every Kp candidate."
        )
    )
    parser.add_argument("motion_program", nargs="?", help="Motion-program YAML path or name.")
    parser.add_argument(
        "--motion-program",
        dest="motion_program_option",
        default=None,
        help="Motion-program YAML path or name. Overrides the positional value.",
    )
    parser.add_argument(
        "--joints",
        default="all",
        help="Joint selection: all, others, or comma-separated joint names. Default: all",
    )
    parser.add_argument(
        "--monitor-joints",
        default="all",
        help="Joints used for task scoring/degradation checks. Default: all",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory for per-joint task tuning CSV files.",
    )
    parser.add_argument(
        "--left-state-topic",
        default="/l_arm_controller/controller_state",
        help="Left arm controller_state topic.",
    )
    parser.add_argument(
        "--right-state-topic",
        default="/r_arm_controller/controller_state",
        help="Right arm controller_state topic.",
    )
    parser.add_argument(
        "--program-edit-service",
        default="/action_recorder/edit",
        help="ProgramEdit service used to load YAML before each trial.",
    )
    parser.add_argument(
        "--program-action",
        default="/action_recorder/command",
        help="ProgramCommand action used to execute play.",
    )
    parser.add_argument(
        "--play-command",
        default="play exec=1 replan=0",
        help="ProgramCommand play command sent after loading the YAML.",
    )
    parser.add_argument("--program-load-timeout", type=float, default=10.0)
    parser.add_argument("--action-send-timeout", type=float, default=10.0)
    parser.add_argument("--play-timeout", type=float, default=300.0)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--post-observe-window", type=float, default=1.0)
    parser.add_argument("--task-repeats", type=int, default=1)
    parser.add_argument("--continue-on-execution-failure", action="store_true")
    parser.add_argument("--continue-on-joint-failure", action="store_true")
    parser.add_argument("--keep-best-kp", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--kp-index", default=DEFAULT_KP_INDEX)
    parser.add_argument("--kp-subindex", type=int, default=DEFAULT_KP_SUBINDEX)
    parser.add_argument("--kp-min", type=float, default=None)
    parser.add_argument("--kp-max", type=float, default=None)
    parser.add_argument("--kp-range", type=float, default=None)
    parser.add_argument("--kp-hard-max", type=float, default=0.0)
    parser.add_argument(
        "--kp-iterations",
        type=int,
        default=12,
        help=f"Candidate Kp evaluations per joint, excluding baseline. Minimum: {AUTO_TUNE_MIN_ITERATIONS}",
    )
    parser.add_argument("--target-rmse-weight", type=float, default=1.0)
    parser.add_argument("--target-mae-weight", type=float, default=0.25)
    parser.add_argument("--target-max-error-weight", type=float, default=0.10)
    parser.add_argument("--target-tail-std-weight", type=float, default=0.10)
    parser.add_argument("--other-degradation-weight", type=float, default=2.0)
    parser.add_argument("--failure-penalty", type=float, default=FAILURE_SCORE)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> tuple[Path, list[str], list[str], Path]:
    motion_program_value = args.motion_program_option or args.motion_program
    if not motion_program_value:
        raise ValueError("Provide a motion-program YAML path or --motion-program")
    if args.kp_iterations < AUTO_TUNE_MIN_ITERATIONS:
        raise ValueError(
            f"--kp-iterations must be at least {AUTO_TUNE_MIN_ITERATIONS}"
        )
    if args.task_repeats <= 0:
        raise ValueError("--task-repeats must be positive")
    if args.play_timeout <= 0.0:
        raise ValueError("--play-timeout must be positive")
    program_path = resolve_motion_program(motion_program_value)
    selected_joints = parse_joint_selection(args.joints, ALL_ARM_JOINTS)
    monitor_joints = parse_joint_selection(args.monitor_joints, selected_joints)
    for joint_name in selected_joints:
        if joint_name not in monitor_joints:
            monitor_joints.append(joint_name)
    output_root = Path(args.output_dir).expanduser()
    return program_path, selected_joints, monitor_joints, output_root


def main() -> int:
    args = parse_args()
    try:
        program_path, selected_joints, monitor_joints, output_root = validate_args(args)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print(f"motion_program: {program_path}")
    print(f"selected_joints: {', '.join(selected_joints)}")
    print(f"monitor_joints: {', '.join(monitor_joints)}")
    print(f"output_dir: {output_root}")

    if args.dry_run:
        return 0

    if not HAS_ROS:
        print(
            "Failed to import ROS 2 dependencies. Source ROS and raybot_core_ws before running.",
            file=sys.stderr,
        )
        print("  source /opt/ros/humble/setup.bash", file=sys.stderr)
        print("  source /home/raybot/raybot_core_ws/install/setup.bash", file=sys.stderr)
        return 1

    rclpy.init()
    client = TaskMotionProgramClient(
        args.program_edit_service,
        args.program_action,
        args.left_state_topic,
        args.right_state_topic,
        monitor_joints,
    )
    summaries: list[JointTuneSummary] = []
    all_summary_csv = output_root / "all_joints_task_tuning_summary.csv"
    return_code = 0
    try:
        client.wait_for_ready(args.state_timeout)
        for joint_name in selected_joints:
            summary = tune_joint(
                client,
                program_path,
                joint_name,
                monitor_joints,
                output_root,
                args,
            )
            summaries.append(summary)
            save_all_joint_summary(all_summary_csv, summaries)
            if summary.status != "success":
                return_code = 1
                if not args.continue_on_joint_failure:
                    break
    finally:
        client.destroy_node()
        rclpy.shutdown()
        if summaries:
            save_all_joint_summary(all_summary_csv, summaries)
            print(f"all_joints_summary_csv: {all_summary_csv}")
    return return_code


if __name__ == "__main__":
    sys.exit(main())
