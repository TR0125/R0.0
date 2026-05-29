#!/usr/bin/env python3
# 指定使用当前环境中的 python3 解释器执行本脚本。

import argparse  # 用于解析命令行参数，例如目标角度、速度、加速度等。
import csv  # 用于把自动调参过程中间 Kp 的评价结果保存为 CSV。
import math  # 用于计算尾均值窗口和若干评价指标。
import re  # 用于从 ethercat 命令输出中提取浮点数。
import shutil  # 用于检查 ethercat 命令是否存在。
import subprocess  # 用于调用 ethercat 读写 Kp 参数。
import sys  # 用于处理退出码，以及将错误信息打印到标准错误输出。
import time  # 用于按单调时钟统计关节阶跃响应的时间指标。
from dataclasses import dataclass, replace  # 用于组织响应测量结果和构造回位请求。
from pathlib import Path  # 用于处理响应曲线图输出路径。
from typing import Optional  # 用于标注可能为空的状态字段。


JOINT_NAME = "right_arm_1_joint"  # 当前脚本控制并监测的目标关节名。
POSITION_EPSILON = 1e-6  # 用于判断“目标已达到/几乎无位移”的最小阈值。
DEFAULT_KP_ALIAS = 1109  # 右臂一关节在 EtherCAT 上的固定 alias。
DEFAULT_KP_INDEX = "0x2006"  # 位置环 P 参数的 SDO index。
DEFAULT_KP_SUBINDEX = 0  # 位置环 P 参数的 SDO subindex。
FLOAT_PATTERN = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")  # 用于从命令输出中提取浮点数字。
RMSE_TARGET_DEG = 0.01  # 目标 RMSE，按角度制指定为 0.01 度，便于和调参目标直接对应。
RMSE_TARGET_RAD = math.radians(RMSE_TARGET_DEG)  # 内部统一换算成弧度参与归一化和达标判断。
PHASE_LAG_TARGET_DEG = 2.0  # 相位滞后目标，按角度制指定为 2 度。
PHASE_LAG_EVAL_FREQUENCY_HZ = 1.0  # 用于把时间延迟换算成相位角的等效频率，默认按 1 Hz 解释。
AMPLITUDE_RATIO_TARGET = 1.0  # 幅值比理想值为 1.0。
AMPLITUDE_RATIO_MIN = 0.95  # 工程上希望幅值比不低于 0.95。
RMSE_EXCESS_PENALTY_WEIGHT = 100.0  # RMSE 超过目标后施加的大罚分，保证搜索优先压低 RMSE。
PHASE_LAG_EXCESS_PENALTY_WEIGHT = 50.0  # 相位滞后超过目标角度后施加的附加罚分。
AMPLITUDE_RATIO_DEFICIT_PENALTY_WEIGHT = 80.0  # 幅值比低于 0.95 后施加的附加罚分。
OVERSHOOT_HARD_PENALTY = 1000.0  # 一旦出现超调就施加硬惩罚，使“无超调”成为近似硬约束。
RMSE_COST_WEIGHT = 1.0  # RMSE 在基础代价中的权重。
PHASE_LAG_COST_WEIGHT = 0.35  # 相位滞后在基础代价中的权重。
AMPLITUDE_RATIO_COST_WEIGHT = 0.35  # 幅值比在基础代价中的权重。
KP_INTERVAL_TOLERANCE = 0.001  # 三分搜索区间宽度小于此值时提前收敛。
KP_MIN_ITERATIONS = 2  # 即使已达标，也至少跑这么多轮再停止。
DEFAULT_KP_RANGE = 0.2  # 自动调参时，在当前 Kp 上下各浮动此值作为默认搜索范围。


@dataclass
class ResponseTracker:  # 跟踪一次 reference/feedback 响应，并计算跟踪误差指标。
    initial_position: float  # 发命令前采集到的关节当前位置。
    initial_reference_position: float  # 发命令前采集到的 reference 目标位置。
    target_position: float  # 本次命令的目标关节位置。
    last_position: Optional[float] = None  # 最近一次收到的关节位置。
    last_reference_position: Optional[float] = None  # 最近一次收到的 reference 目标位置。
    command_time: Optional[float] = None  # 发出运动命令的单调时钟时间，作为曲线图横轴零点。
    sample_times: list[float] = None  # 记录每个反馈样本相对命令下发的时间。
    sample_reference_positions: list[float] = None  # 记录每个样本对应的 reference 目标位置。
    sample_positions: list[float] = None  # 记录每个反馈样本的关节位置。
    sample_errors: list[float] = None  # 记录每个样本的跟踪误差，定义为 feedback - reference。

    def __post_init__(self) -> None:  # 初始化用于绘图的样本缓存。
        self.sample_times = []
        self.sample_reference_positions = []
        self.sample_positions = []
        self.sample_errors = []

    def has_motion(self) -> bool:  # 判断这次命令是否真的引起了 reference 变化。
        return abs(self.target_position - self.initial_position) > POSITION_EPSILON

    def update(self, reference_position: float, position: float, now: float) -> None:  # 用新的 reference 和 feedback 更新响应采样。
        self.last_position = position  # 始终保存最后一帧位置，便于最终输出。
        self.last_reference_position = reference_position  # 始终保存最后一帧目标位置，便于最终输出。
        if self.command_time is not None:  # 只有命令真正发出后，才把反馈写入响应曲线样本。
            self.sample_times.append(now - self.command_time)
            self.sample_reference_positions.append(reference_position)
            self.sample_positions.append(position)
            self.sample_errors.append(position - reference_position)

    def rmse(self) -> Optional[float]:  # 返回误差序列的均方根误差。
        if not self.sample_errors:
            return None
        mean_square = sum(error * error for error in self.sample_errors) / len(self.sample_errors)
        return mean_square ** 0.5

    def set_command_time(self, now: float) -> None:  # 记录命令下发时刻，并从该时刻开始生成响应曲线横轴。
        self.command_time = now
        self.sample_times = [0.0]
        self.sample_reference_positions = [self.initial_reference_position]
        self.sample_positions = [self.initial_position]
        self.sample_errors = [self.initial_position - self.initial_reference_position]


@dataclass
class TrackingMetrics:  # 保存一次跟踪试验的评价指标和总代价。
    rmse: float  # 整个观测窗口内的误差均方根。
    phase_lag_deg: float  # 按等效频率换算得到的相位滞后角度。
    amplitude_ratio: float  # 输出幅值 / 输入幅值 的稳态幅值比。
    overshoot_rad: float  # 相对最终 reference 的绝对超调量。
    rmse_term: float  # RMSE 按目标值 0.01 度归一化后的代价值分量。
    phase_lag_term: float  # 相位滞后按目标角度归一化后的代价值分量。
    amplitude_ratio_term: float  # 幅值比相对理想值的代价值分量。
    rmse_penalty: float  # RMSE 超目标后的附加罚分。
    phase_lag_penalty: float  # 相位滞后超目标角度后的附加罚分。
    amplitude_ratio_penalty: float  # 幅值比低于 0.95 后的附加罚分。
    overshoot_penalty: float  # 出现超调后的硬惩罚。
    cost: float  # 各项指标按权重折算后的总代价。
    meets_target: bool  # 是否同时满足 RMSE 和无超调目标。
    note: str = ""  # 记录异常情况或额外说明。


@dataclass
class KpTrialResult:  # 保存某个 Kp 候选值的一次试验结果。
    kp: float  # 本次试验写入的位置环 P 参数。
    tracker: ResponseTracker  # 本次试验对应的 reference/feedback 响应曲线样本。
    metrics: TrackingMetrics  # 本次试验对应的评价指标。


@dataclass
class MotionRequest:  # 单次运动试验的参数配置，替代原地修改 argparse.Namespace。
    position: float  # 目标关节位置，单位 rad。
    vel: float = 0.3  # 速度缩放系数。
    acc: float = 0.3  # 加速度缩放系数。
    pipeline: str = "ompl"  # 规划 pipeline。
    planner_id: str = "RRTConnectkConfigDefault"  # 规划器 ID。
    exec_motion: bool = True  # 是否真正执行运动，为 False 时只规划不执行。
    no_wait: bool = False  # 是否不等待服务响应。
    timeout: float = 0.0  # 服务请求超时，单位秒。
    spin_timeout: float = 10.0  # 本地等待服务响应的超时，单位秒。
    state_timeout: float = 2.0  # 等待初始控制器状态的超时，单位秒。
    observe_window: float = 3.0  # 命令发出后继续采样的时间，单位秒。


@dataclass
class AutoTuneResult:  # 自动调参的完整结果，替代多元组返回。
    best_kp: float  # 搜索到的最优 Kp。
    history: list  # 所有中间候选 Kp 的试验结果列表。
    original_kp: float  # 调参前的原始 Kp。
    baseline_result: "KpTrialResult"  # 原始 Kp 的基线试验结果。
    reset_position: float  # 基线试验时使用的统一起始位置。


def find_reference_change_index(
    reference_positions: list[float],
    baseline: float,
) -> Optional[int]:  # 找到 reference 首次明显偏离初始值的样本索引。
    for index, value in enumerate(reference_positions):
        if abs(value - baseline) > POSITION_EPSILON:
            return index
    return None


def find_max_abs_error_index(errors: list[float]) -> Optional[int]:  # 找到最大绝对误差对应的样本索引。
    if not errors:
        return None
    return max(range(len(errors)), key=lambda index: abs(errors[index]))


def tail_mean(values: list[float], ratio: float = 0.2) -> float:  # 用末尾一段样本均值估计稳态值，降低单点噪声影响。
    if not values:
        raise ValueError("cannot compute tail mean from an empty list")
    window = max(1, math.ceil(len(values) * ratio))
    tail = values[-window:]
    return sum(tail) / len(tail)


def first_crossing_time(
    times: list[float],
    values: list[float],
    threshold: float,
    direction: float,
) -> Optional[float]:  # 找到序列首次越过阈值的时间，用于估计 reference/feedback 的相对滞后。
    if len(times) != len(values) or not times:
        return None
    for index, value in enumerate(values):
        if direction >= 0.0 and value >= threshold:
            return times[index]
        if direction < 0.0 and value <= threshold:
            return times[index]
    return None


def compute_tracking_metrics(tracker: ResponseTracker) -> TrackingMetrics:  # 基于采样到的 reference/feedback 计算单次试验的评价指标。
    rmse = tracker.rmse()
    if rmse is None:
        raise RuntimeError("No tracking samples available to compute metrics")

    initial_reference = tracker.sample_reference_positions[0]
    final_reference = tail_mean(tracker.sample_reference_positions)  # 用 reference 末端均值估计最终目标。
    final_feedback = tail_mean(tracker.sample_positions)  # 用 feedback 末端均值估计最终实际位置。
    reference_step = final_reference - initial_reference  # reference 的有效阶跃幅值。

    if abs(reference_step) <= POSITION_EPSILON:  # reference 没怎么变化时，无法稳定定义相位/幅值指标。
        return TrackingMetrics(
            rmse=rmse,
            phase_lag_deg=float("inf"),
            amplitude_ratio=0.0,
            overshoot_rad=0.0,
            rmse_term=rmse / RMSE_TARGET_RAD,
            phase_lag_term=float("inf"),
            amplitude_ratio_term=float("inf"),
            rmse_penalty=RMSE_EXCESS_PENALTY_WEIGHT * max(0.0, rmse / RMSE_TARGET_RAD - 1.0),
            phase_lag_penalty=float("inf"),
            amplitude_ratio_penalty=float("inf"),
            overshoot_penalty=0.0,
            cost=float("inf"),
            meets_target=False,
            note="reference step too small",
        )

    reference_threshold = initial_reference + 0.5 * reference_step  # 用 50% 交叉点近似阶跃相位滞后。
    reference_cross = first_crossing_time(
        tracker.sample_times,
        tracker.sample_reference_positions,
        reference_threshold,
        reference_step,
    )
    feedback_cross = first_crossing_time(
        tracker.sample_times,
        tracker.sample_positions,
        reference_threshold,
        reference_step,
    )
    phase_lag_sec = (
        feedback_cross - reference_cross
        if reference_cross is not None and feedback_cross is not None
        else float("inf")
    )
    phase_lag_deg = (
        phase_lag_sec * 360.0 * PHASE_LAG_EVAL_FREQUENCY_HZ
        if math.isfinite(phase_lag_sec)
        else float("inf")
    )  # 按 1Hz 等效频率把时间滞后换算成相位角度。

    amplitude_ratio = (final_feedback - tracker.initial_position) / reference_step  # 实际末端响应幅值与 reference 幅值之比。
    amplitude_ratio_error = abs(AMPLITUDE_RATIO_TARGET - amplitude_ratio)  # 与理想幅值比 1.0 的偏差。

    if reference_step > 0.0:  # 按 reference 最终值定义超调。
        peak_feedback = max(tracker.sample_positions)
        overshoot_rad = max(0.0, peak_feedback - final_reference)
    else:
        trough_feedback = min(tracker.sample_positions)
        overshoot_rad = max(0.0, final_reference - trough_feedback)

    rmse_term = rmse / RMSE_TARGET_RAD  # 把 RMSE 归一化到“0.01 度目标值的多少倍”。
    phase_lag_term = (
        phase_lag_deg / PHASE_LAG_TARGET_DEG
        if math.isfinite(phase_lag_deg)
        else 1_000.0
    )  # 把相位滞后归一化到 2 度目标的倍数；不可定义时直接记为极大值。
    amplitude_ratio_term = amplitude_ratio_error / (1.0 - AMPLITUDE_RATIO_MIN)  # 把幅值比偏差归一化到 0.95 下限的容忍带宽。

    base_cost = (
        RMSE_COST_WEIGHT * rmse_term
        + PHASE_LAG_COST_WEIGHT * phase_lag_term
        + AMPLITUDE_RATIO_COST_WEIGHT * amplitude_ratio_term
    )
    rmse_penalty = RMSE_EXCESS_PENALTY_WEIGHT * max(0.0, rmse_term - 1.0)  # RMSE 超过 0.01 度目标后，按超出比例施加大罚分。
    phase_lag_penalty = PHASE_LAG_EXCESS_PENALTY_WEIGHT * max(0.0, phase_lag_term - 1.0)  # 相位滞后超过 2 度目标后，按超出比例施加罚分。
    amplitude_ratio_penalty = AMPLITUDE_RATIO_DEFICIT_PENALTY_WEIGHT * max(0.0, (AMPLITUDE_RATIO_MIN - amplitude_ratio) / AMPLITUDE_RATIO_MIN)  # 幅值比低于 0.95 后施加罚分。
    overshoot_penalty = OVERSHOOT_HARD_PENALTY if overshoot_rad > POSITION_EPSILON else 0.0  # 一旦出现超调，就直接让总代价大幅上升。
    cost = base_cost + rmse_penalty + phase_lag_penalty + amplitude_ratio_penalty + overshoot_penalty
    meets_target = (
        rmse <= RMSE_TARGET_RAD
        and phase_lag_deg <= PHASE_LAG_TARGET_DEG
        and amplitude_ratio >= AMPLITUDE_RATIO_MIN
        and overshoot_rad <= POSITION_EPSILON
    )  # 满足目标：RMSE<=0.01 度、相位滞后<=2 度、幅值比>=0.95 且无超调。

    return TrackingMetrics(
        rmse=rmse,
        phase_lag_deg=phase_lag_deg,
        amplitude_ratio=amplitude_ratio,
        overshoot_rad=overshoot_rad,
        rmse_term=rmse_term,
        phase_lag_term=phase_lag_term,
        amplitude_ratio_term=amplitude_ratio_term,
        rmse_penalty=rmse_penalty,
        phase_lag_penalty=phase_lag_penalty,
        amplitude_ratio_penalty=amplitude_ratio_penalty,
        overshoot_penalty=overshoot_penalty,
        cost=cost,
        meets_target=meets_target,
    )


def ensure_ethercat_available() -> None:  # 在真正调参前检查 ethercat 命令是否存在。
    if shutil.which("ethercat") is None:
        raise RuntimeError("ethercat command not found in PATH")


def run_ethercat_command(args: list[str]) -> str:  # 统一封装 ethercat 命令调用，并在失败时给出明确错误。
    completed = subprocess.run(
        args,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip()
        stdout = completed.stdout.strip()
        detail = stderr or stdout or f"exit code {completed.returncode}"
        raise RuntimeError(f"{' '.join(args)} failed: {detail}")
    return completed.stdout.strip()


def parse_float_from_output(text: str) -> float:  # 从 ethercat upload 的输出中提取一个浮点数。
    match = FLOAT_PATTERN.search(text)
    if match is None:
        raise RuntimeError(f"Could not parse float from output: {text!r}")
    return float(match.group(0))


def read_kp(alias: int, index: str, subindex: int) -> float:  # 读取当前位置环 Kp。
    ensure_ethercat_available()
    output = run_ethercat_command(
        ["ethercat", "upload", "-a", str(alias), "-t", "float", index, str(subindex)]
    )
    return parse_float_from_output(output)


def write_kp(alias: int, index: str, subindex: int, value: float) -> None:  # 写入当前位置环 Kp。
    ensure_ethercat_available()
    run_ethercat_command(
        [
            "ethercat",
            "download",
            "-a",
            str(alias),
            "-t",
            "float",
            index,
            str(subindex),
            f"{value:.6f}",
        ]
    )

def default_plot_path(target_position: float) -> str:  # 生成默认响应曲线图文件名，避免每次都手工指定路径。
    safe_target = str(target_position).replace("-", "neg_").replace(".", "_")
    return str(Path(__file__).with_name(f"right_arm_1_joint_response_{safe_target}.png"))


def _expand_range(min_value: float, max_value: float) -> tuple[float, float]:  # 为绘图坐标轴补一点边距，避免曲线贴边。
    if min_value == max_value:
        padding = 1.0 if abs(min_value) < 1.0 else abs(min_value) * 0.1
        return min_value - padding, max_value + padding
    padding = (max_value - min_value) * 0.08
    return min_value - padding, max_value + padding


def _polyline_points(xs: list[float], ys: list[float], x_map, y_map) -> str:  # 把样本点映射成 SVG polyline 所需的点串。
    return " ".join(f"{x_map(x):.2f},{y_map(y):.2f}" for x, y in zip(xs, ys))


def _linspace_ticks(start: float, end: float, count: int) -> list[float]:  # 生成等间隔刻度，供 SVG 版本画网格和坐标值。
    if count <= 1:
        return [start]
    step = (end - start) / (count - 1)
    return [start + step * index for index in range(count)]


def _annotation_style(color: str) -> dict:  # 统一标注样式，增加浅色底和引导线，提升密集区域可读性。
    return {
        "color": color,
        "fontsize": 9,
        "bbox": {"boxstyle": "round,pad=0.18", "fc": "white", "ec": "none", "alpha": 0.78},
        "arrowprops": {"arrowstyle": "-", "color": color, "lw": 0.9, "alpha": 0.7},
    }


def _position_annotation_offsets(
    tracker: ResponseTracker,
    reference_change_index: Optional[int],
) -> dict[str, tuple[int, int]]:  # 根据起点/变更点相对位置自动错开标注，避免 start ref、start fb、reference change 重叠。
    offsets = {
        "start_ref": (8, -14),
        "start_fb": (8, 10),
        "reference_change": (8, -14),
        "end": (-28, -14),
    }
    all_positions = tracker.sample_reference_positions + tracker.sample_positions
    if not all_positions:
        return offsets

    x_span = max(tracker.sample_times) - min(tracker.sample_times) if tracker.sample_times else 0.0
    y_span = max(all_positions) - min(all_positions)
    x_threshold = max(x_span * 0.05, 0.05)
    y_threshold = max(y_span * 0.05, 1e-4)

    start_time = tracker.sample_times[0]
    start_ref = tracker.sample_reference_positions[0]
    start_fb = tracker.sample_positions[0]
    start_points_close = (
        abs(start_ref - start_fb) <= y_threshold
    )
    if start_points_close:
        offsets["start_ref"] = (10, -16)
        offsets["start_fb"] = (10, 14)

    if reference_change_index is None:
        return offsets

    change_time = tracker.sample_times[reference_change_index]
    change_ref = tracker.sample_reference_positions[reference_change_index]
    change_near_start_ref = (
        abs(change_time - start_time) <= x_threshold
        and abs(change_ref - start_ref) <= y_threshold
    )
    change_near_start_fb = (
        abs(change_time - start_time) <= x_threshold
        and abs(change_ref - start_fb) <= y_threshold
    )

    if change_near_start_ref and change_near_start_fb:
        offsets["reference_change"] = (42, 10)
    elif change_near_start_ref:
        offsets["reference_change"] = (14, 12)
    elif change_near_start_fb:
        offsets["reference_change"] = (14, -18)

    return offsets


def _comparison_position_limits(
    baseline: KpTrialResult,
    best: KpTrialResult,
) -> tuple[tuple[float, float], tuple[float, float]]:  # 为基线和最优位置图计算统一坐标范围，保证对比呈现一致。
    time_values = baseline.tracker.sample_times + best.tracker.sample_times
    position_values = (
        baseline.tracker.sample_reference_positions
        + baseline.tracker.sample_positions
        + best.tracker.sample_reference_positions
        + best.tracker.sample_positions
        + [baseline.tracker.initial_position, baseline.tracker.target_position]
        + [best.tracker.initial_position, best.tracker.target_position]
    )
    return _expand_range(min(time_values), max(time_values)), _expand_range(
        min(position_values),
        max(position_values),
    )


def _comparison_error_limits(
    baseline: KpTrialResult,
    best: KpTrialResult,
) -> tuple[tuple[float, float], tuple[float, float]]:  # 为基线和最优误差图计算统一坐标范围，便于横向比较误差量级。
    time_values = baseline.tracker.sample_times + best.tracker.sample_times
    error_values = baseline.tracker.sample_errors + best.tracker.sample_errors
    return _expand_range(min(time_values), max(time_values)), _expand_range(
        min(error_values),
        max(error_values),
    )


def default_kp_trials_csv_path() -> str:  # 生成默认的 Kp 调参记录 CSV 路径，固定存放在当前脚本目录。
    return str(Path(__file__).with_name("kp_trials.csv"))


def save_kp_trials_csv(
    baseline: KpTrialResult,
    history: list[KpTrialResult],
    best_kp: float,
    output_path: str,
) -> str:  # 将原始 Kp 与所有中间候选 Kp 的评价结果写入 CSV，便于后续筛选与分析。
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = [baseline] + history
    fieldnames = [
        "role",
        "kp",
        "rmse_rad",
        "phase_lag_deg",
        "amplitude_ratio",
        "overshoot_rad",
        "rmse_term",
        "phase_lag_term",
        "amplitude_ratio_term",
        "rmse_penalty",
        "phase_lag_penalty",
        "amplitude_ratio_penalty",
        "overshoot_penalty",
        "cost",
        "meets_target",
        "note",
        "initial_position_rad",
        "initial_reference_position_rad",
        "target_position_rad",
        "final_position_rad",
        "final_reference_position_rad",
        "sample_count",
    ]
    with output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for index, trial in enumerate(rows):
            role = "baseline" if index == 0 else ("best_candidate" if abs(trial.kp - best_kp) <= POSITION_EPSILON else "candidate")
            writer.writerow(
                {
                    "role": role,
                    "kp": f"{trial.kp:.6f}",
                    "rmse_rad": f"{trial.metrics.rmse:.9f}",
                    "phase_lag_deg": f"{trial.metrics.phase_lag_deg:.9f}",
                    "amplitude_ratio": f"{trial.metrics.amplitude_ratio:.9f}",
                    "overshoot_rad": f"{trial.metrics.overshoot_rad:.9f}",
                    "rmse_term": f"{trial.metrics.rmse_term:.9f}",
                    "phase_lag_term": f"{trial.metrics.phase_lag_term:.9f}",
                    "amplitude_ratio_term": f"{trial.metrics.amplitude_ratio_term:.9f}",
                    "rmse_penalty": f"{trial.metrics.rmse_penalty:.9f}",
                    "phase_lag_penalty": f"{trial.metrics.phase_lag_penalty:.9f}",
                    "amplitude_ratio_penalty": f"{trial.metrics.amplitude_ratio_penalty:.9f}",
                    "overshoot_penalty": f"{trial.metrics.overshoot_penalty:.9f}",
                    "cost": f"{trial.metrics.cost:.9f}",
                    "meets_target": str(trial.metrics.meets_target),
                    "note": trial.metrics.note,
                    "initial_position_rad": f"{trial.tracker.initial_position:.9f}",
                    "initial_reference_position_rad": f"{trial.tracker.initial_reference_position:.9f}",
                    "target_position_rad": f"{trial.tracker.target_position:.9f}",
                    "final_position_rad": (
                        f"{trial.tracker.last_position:.9f}" if trial.tracker.last_position is not None else ""
                    ),
                    "final_reference_position_rad": (
                        f"{trial.tracker.last_reference_position:.9f}"
                        if trial.tracker.last_reference_position is not None
                        else ""
                    ),
                    "sample_count": len(trial.tracker.sample_times),
                }
            )
    return str(output)


def extract_position(values, index: int) -> float:  # 从 controller_state 的 positions 数组中按索引取目标关节值。
    if index < 0 or index >= len(values):
        raise ValueError("positions array does not contain the requested joint index")
    return float(values[index])


def save_response_plot_svg(
    tracker: ResponseTracker,
    output_path: str,
    rmse: Optional[float],
) -> str:  # 在没有 matplotlib 时，用标准库生成 SVG 跟踪响应图。
    if not tracker.sample_times or not tracker.sample_positions:  # 没有样本时直接报错，避免生成空图。
        raise RuntimeError("No response samples collected for plotting")

    output = Path(output_path)  # 统一按 Path 处理，便于自动创建父目录和调整扩展名。
    if output.suffix.lower() != ".svg":  # 无 matplotlib 时改存为 SVG，避免伪造 PNG 扩展名。
        output = output.with_suffix(".svg")

    width = 1000  # 整张 SVG 画布宽度。
    height = 720  # 整张 SVG 画布高度。
    margin_left = 80  # 左边距，用于放坐标轴和标签。
    margin_right = 28  # 右边距。
    margin_top = 64  # 顶部边距，用于放标题。
    margin_bottom = 56  # 底部边距，用于放横轴标签。
    gap = 44  # 两张子图之间的间距。
    panel_width = width - margin_left - margin_right  # 单个子图的可绘制宽度。
    panel_height = (height - margin_top - margin_bottom - gap) / 2.0  # 每个子图的可绘制高度。

    time_min, time_max = _expand_range(min(tracker.sample_times), max(tracker.sample_times))  # 计算时间轴范围。
    position_min, position_max = _expand_range(
        min(min(tracker.sample_positions), tracker.initial_position, tracker.target_position),
        max(max(tracker.sample_positions), tracker.initial_position, tracker.target_position),
    )  # 计算位置轴范围，确保目标线和初值都在图内。
    error_min, error_max = _expand_range(
        min(tracker.sample_errors),
        max(tracker.sample_errors),
    )  # 计算误差轴范围。

    def map_x(value: float) -> float:  # 把时间值映射到 SVG 横坐标。
        return margin_left + (value - time_min) / (time_max - time_min) * panel_width

    def map_pos_y(value: float) -> float:  # 把位置值映射到上半图纵坐标。
        return margin_top + panel_height - (value - position_min) / (position_max - position_min) * panel_height

    def map_error_y(value: float) -> float:  # 把误差值映射到下半图纵坐标。
        base = margin_top + panel_height + gap
        return base + panel_height - (value - error_min) / (error_max - error_min) * panel_height

    position_points = _polyline_points(
        tracker.sample_times,
        tracker.sample_positions,
        map_x,
        map_pos_y,
    )  # 把位置响应样本拼成 SVG 折线。
    reference_points = _polyline_points(
        tracker.sample_times,
        tracker.sample_reference_positions,
        map_x,
        map_pos_y,
    )  # 把 reference 目标样本拼成 SVG 折线。
    error_points = _polyline_points(
        tracker.sample_times,
        tracker.sample_errors,
        map_x,
        map_error_y,
    )  # 把误差样本拼成 SVG 折线。

    title_parts = [f"{JOINT_NAME} tracking response"]  # 在标题里汇总本次试验最关键的跟踪指标。
    if rmse is not None:
        title_parts.append(f"RMSE={rmse:.6f}rad")
    title = " | ".join(title_parts)

    bottom_panel_top = margin_top + panel_height + gap  # 下半图起始纵坐标。
    bottom_panel_bottom = bottom_panel_top + panel_height  # 下半图结束纵坐标。
    x_ticks = _linspace_ticks(time_min, time_max, 6)  # 为时间轴生成主刻度。
    position_ticks = _linspace_ticks(position_min, position_max, 6)  # 为位置轴生成主刻度。
    error_ticks = _linspace_ticks(error_min, error_max, 6)  # 为误差轴生成主刻度。
    reference_change_index = find_reference_change_index(
        tracker.sample_reference_positions,
        tracker.initial_reference_position,
    )  # 定位 reference 真正开始变化的时刻。
    max_error_index = find_max_abs_error_index(tracker.sample_errors)  # 定位最大绝对误差对应的样本。

    position_grid = []  # 收集上半图网格线和刻度文本。
    for tick in x_ticks:
        x = map_x(tick)
        position_grid.append(
            f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" y2="{margin_top + panel_height}" '
            f'stroke="#d9d9d9" stroke-width="1" />'
        )  # 画上半图竖向网格线。
    for tick in position_ticks:
        y = map_pos_y(tick)
        position_grid.append(
            f'<line x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + panel_width}" y2="{y:.2f}" '
            f'stroke="#d9d9d9" stroke-width="1" />'
        )  # 画上半图横向网格线。
        position_grid.append(
            f'<text x="{margin_left - 10}" y="{y + 4:.2f}" text-anchor="end" font-size="11" '
            f'font-family="DejaVu Sans, Arial, sans-serif" fill="#444">{tick:.4f}</text>'
        )  # 在左侧标出位置刻度值。

    error_grid = []  # 收集下半图网格线和刻度文本。
    for tick in x_ticks:
        x = map_x(tick)
        error_grid.append(
            f'<line x1="{x:.2f}" y1="{bottom_panel_top}" x2="{x:.2f}" y2="{bottom_panel_bottom}" '
            f'stroke="#d9d9d9" stroke-width="1" />'
        )  # 画下半图竖向网格线。
        error_grid.append(
            f'<text x="{x:.2f}" y="{height - 34}" text-anchor="middle" font-size="11" '
            f'font-family="DejaVu Sans, Arial, sans-serif" fill="#444">{tick:.3f}</text>'
        )  # 在底部标出时间刻度值。
    for tick in error_ticks:
        y = map_error_y(tick)
        error_grid.append(
            f'<line x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + panel_width}" y2="{y:.2f}" '
            f'stroke="#d9d9d9" stroke-width="1" />'
        )  # 画下半图横向网格线。
        error_grid.append(
            f'<text x="{margin_left - 10}" y="{y + 4:.2f}" text-anchor="end" font-size="11" '
            f'font-family="DejaVu Sans, Arial, sans-serif" fill="#444">{tick:.4f}</text>'
        )  # 在左侧标出误差刻度值。

    position_markers = []  # 收集上图关键节点标记。
    error_markers = []  # 收集下图关键节点标记。

    start_x = map_x(tracker.sample_times[0])
    start_ref_y = map_pos_y(tracker.sample_reference_positions[0])
    start_fb_y = map_pos_y(tracker.sample_positions[0])
    position_markers.append(
        f'<circle cx="{start_x:.2f}" cy="{start_ref_y:.2f}" r="4.5" fill="#d62728" />'
    )
    position_markers.append(
        f'<text x="{start_x + 8:.2f}" y="{start_ref_y - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#d62728">start ref</text>'
    )
    position_markers.append(
        f'<circle cx="{start_x:.2f}" cy="{start_fb_y:.2f}" r="4.5" fill="#1f77b4" />'
    )
    position_markers.append(
        f'<text x="{start_x + 8:.2f}" y="{start_fb_y + 16:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#1f77b4">start fb</text>'
    )
    error_markers.append(
        f'<circle cx="{start_x:.2f}" cy="{map_error_y(tracker.sample_errors[0]):.2f}" r="4.5" fill="#2ca02c" />'
    )
    error_markers.append(
        f'<text x="{start_x + 8:.2f}" y="{map_error_y(tracker.sample_errors[0]) - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#2ca02c">start</text>'
    )

    if reference_change_index is not None:  # 若 reference 在观测窗口内发生了变化，则标注其首次变化点。
        change_x = map_x(tracker.sample_times[reference_change_index])
        change_y = map_pos_y(tracker.sample_reference_positions[reference_change_index])
        position_markers.append(
            f'<circle cx="{change_x:.2f}" cy="{change_y:.2f}" r="5" fill="#ff7f0e" />'
        )
        position_markers.append(
            f'<text x="{change_x + 8:.2f}" y="{change_y - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#ff7f0e">reference change</text>'
        )

    if max_error_index is not None:  # 标注最大绝对误差点，便于快速判断最差跟踪时刻。
        max_error_x = map_x(tracker.sample_times[max_error_index])
        max_error_y = map_error_y(tracker.sample_errors[max_error_index])
        error_markers.append(
            f'<circle cx="{max_error_x:.2f}" cy="{max_error_y:.2f}" r="5" fill="#9467bd" />'
        )
        error_markers.append(
            f'<text x="{max_error_x + 8:.2f}" y="{max_error_y - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#9467bd">max |error|</text>'
        )

    end_index = len(tracker.sample_times) - 1  # 最后一个样本作为终点。
    end_x = map_x(tracker.sample_times[end_index])
    end_ref_y = map_pos_y(tracker.sample_reference_positions[end_index])
    end_fb_y = map_pos_y(tracker.sample_positions[end_index])
    position_markers.append(
        f'<circle cx="{end_x:.2f}" cy="{end_ref_y:.2f}" r="4.5" fill="#d62728" />'
    )
    position_markers.append(
        f'<circle cx="{end_x:.2f}" cy="{end_fb_y:.2f}" r="4.5" fill="#1f77b4" />'
    )
    position_markers.append(
        f'<text x="{end_x - 36:.2f}" y="{end_fb_y - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#1f77b4">end</text>'
    )
    error_markers.append(
        f'<circle cx="{end_x:.2f}" cy="{map_error_y(tracker.sample_errors[end_index]):.2f}" r="4.5" fill="#2ca02c" />'
    )
    error_markers.append(
        f'<text x="{end_x - 28:.2f}" y="{map_error_y(tracker.sample_errors[end_index]) - 8:.2f}" font-size="11" font-family="DejaVu Sans, Arial, sans-serif" fill="#2ca02c">end</text>'
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white" />
  <text x="{width / 2:.1f}" y="30" text-anchor="middle" font-size="20" font-family="DejaVu Sans, Arial, sans-serif">{title}</text>

  <rect x="{margin_left}" y="{margin_top}" width="{panel_width}" height="{panel_height}" fill="none" stroke="#333" stroke-width="1" />
  {' '.join(position_grid)}
  <polyline fill="none" stroke="#d62728" stroke-width="2" points="{reference_points}" />
  <polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{position_points}" />
  {' '.join(position_markers)}
  <text x="16" y="{margin_top + panel_height / 2:.1f}" transform="rotate(-90 16,{margin_top + panel_height / 2:.1f})" font-size="14" font-family="DejaVu Sans, Arial, sans-serif">Position (rad)</text>

  <rect x="{margin_left}" y="{bottom_panel_top}" width="{panel_width}" height="{panel_height}" fill="none" stroke="#333" stroke-width="1" />
  {' '.join(error_grid)}
  <line x1="{margin_left}" y1="{map_error_y(0.0):.2f}" x2="{margin_left + panel_width}" y2="{map_error_y(0.0):.2f}" stroke="#666" stroke-width="1.2" stroke-dasharray="8 6" />
  <polyline fill="none" stroke="#2ca02c" stroke-width="2" points="{error_points}" />
  {' '.join(error_markers)}
  <text x="16" y="{bottom_panel_top + panel_height / 2:.1f}" transform="rotate(-90 16,{bottom_panel_top + panel_height / 2:.1f})" font-size="14" font-family="DejaVu Sans, Arial, sans-serif">Error (rad)</text>
  <text x="{margin_left + panel_width / 2:.1f}" y="{height - 18}" text-anchor="middle" font-size="14" font-family="DejaVu Sans, Arial, sans-serif">Time since command (s)</text>

  <text x="{margin_left + 8}" y="{margin_top + 18}" font-size="12" fill="#d62728" font-family="DejaVu Sans, Arial, sans-serif">reference</text>
  <text x="{margin_left + 92}" y="{margin_top + 18}" font-size="12" fill="#1f77b4" font-family="DejaVu Sans, Arial, sans-serif">feedback</text>
  <text x="{margin_left + 8}" y="{bottom_panel_top + 18}" font-size="12" fill="#2ca02c" font-family="DejaVu Sans, Arial, sans-serif">tracking error</text>
</svg>
"""

    output.parent.mkdir(parents=True, exist_ok=True)  # 若用户指定了不存在的目录，则自动创建。
    output.write_text(svg, encoding="utf-8")  # 把 SVG 文本写入文件。
    return str(output)  # 返回实际保存路径，便于主流程打印。


def save_response_plot(
    tracker: ResponseTracker,
    output_path: str,
    rmse: Optional[float],
) -> str:  # 优先用 matplotlib 生成图片，缺失时回退到标准库 SVG。
    if not tracker.sample_times or not tracker.sample_positions:  # 没有样本时直接报错，避免生成空图。
        raise RuntimeError("No response samples collected for plotting")

    try:  # 按需导入绘图库，避免在不画图时增加无谓依赖。
        import matplotlib.pyplot as plt
    except ImportError:
        return save_response_plot_svg(  # 若 matplotlib 不可用，则回退到无依赖的 SVG 绘图实现。
            tracker,
            output_path,
            rmse,
        )

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)  # 上图画位置响应，下图画跟踪误差。
    position_axis, error_axis = axes
    reference_change_index = find_reference_change_index(
        tracker.sample_reference_positions,
        tracker.initial_reference_position,
    )  # 定位 reference 真正开始变化的时刻。
    max_error_index = find_max_abs_error_index(tracker.sample_errors)  # 定位最大绝对误差样本。

    position_axis.plot(
        tracker.sample_times,
        tracker.sample_reference_positions,
        label="reference",
        color="#d62728",
        linewidth=2.0,
    )  # 绘制控制器 reference 目标曲线。
    position_axis.plot(
        tracker.sample_times,
        tracker.sample_positions,
        label="feedback",
        color="#1f77b4",
        linewidth=2.0,
    )  # 绘制控制器 feedback 实际位置响应曲线。
    position_axis.set_ylabel("Position (rad)")  # 设置纵轴标签。
    position_axis.minorticks_on()  # 打开次刻度，增强网格密度。
    position_axis.grid(True, which="major", linestyle="--", alpha=0.45)  # 添加主网格，方便人工读图。
    position_axis.grid(True, which="minor", linestyle=":", alpha=0.2)  # 添加次网格，提升读图精度。
    position_axis.legend(loc="best")  # 显示图例。
    position_axis.tick_params(axis="both", labelsize=10)  # 明确显示横纵坐标刻度文字。
    position_axis.scatter(
        [tracker.sample_times[0]],
        [tracker.sample_reference_positions[0]],
        color="#d62728",
        s=35,
        zorder=5,
    )
    position_offsets = _position_annotation_offsets(tracker, reference_change_index)
    position_axis.annotate(
        "start ref",
        (tracker.sample_times[0], tracker.sample_reference_positions[0]),
        xytext=position_offsets["start_ref"],
        textcoords="offset points",
        **_annotation_style("#d62728"),
    )
    position_axis.scatter(
        [tracker.sample_times[0]],
        [tracker.sample_positions[0]],
        color="#1f77b4",
        s=35,
        zorder=5,
    )
    position_axis.annotate(
        "start fb",
        (tracker.sample_times[0], tracker.sample_positions[0]),
        xytext=position_offsets["start_fb"],
        textcoords="offset points",
        **_annotation_style("#1f77b4"),
    )
    if reference_change_index is not None:
        position_axis.scatter(
            [tracker.sample_times[reference_change_index]],
            [tracker.sample_reference_positions[reference_change_index]],
            color="#ff7f0e",
            s=42,
            zorder=5,
        )
        position_axis.annotate(
            "reference change",
            (
                tracker.sample_times[reference_change_index],
                tracker.sample_reference_positions[reference_change_index],
            ),
            xytext=position_offsets["reference_change"],
            textcoords="offset points",
            **_annotation_style("#ff7f0e"),
        )
    end_index = len(tracker.sample_times) - 1
    position_axis.scatter(
        [tracker.sample_times[end_index]],
        [tracker.sample_positions[end_index]],
        color="#1f77b4",
        s=35,
        zorder=5,
    )
    position_axis.annotate(
        "end",
        (tracker.sample_times[end_index], tracker.sample_positions[end_index]),
        xytext=position_offsets["end"],
        textcoords="offset points",
        **_annotation_style("#1f77b4"),
    )

    error_axis.plot(
        tracker.sample_times,
        tracker.sample_errors,
        label="tracking error",
        color="#2ca02c",
        linewidth=2.0,
    )  # 绘制 tracking error 曲线。
    error_axis.axhline(0.0, color="#666", linestyle="--", linewidth=1.2)  # 绘制零误差参考线。
    error_axis.set_ylabel("Error (rad)")  # 设置纵轴标签。
    error_axis.set_xlabel("Time since command (s)")  # 设置横轴标签。
    error_axis.minorticks_on()  # 打开次刻度，增强网格密度。
    error_axis.grid(True, which="major", linestyle="--", alpha=0.45)  # 添加主网格。
    error_axis.grid(True, which="minor", linestyle=":", alpha=0.2)  # 添加次网格。
    error_axis.tick_params(axis="both", labelsize=10)  # 明确显示横纵坐标刻度文字。
    error_axis.legend(loc="best")  # 显示误差图例。
    error_axis.scatter(
        [tracker.sample_times[0]],
        [tracker.sample_errors[0]],
        color="#2ca02c",
        s=35,
        zorder=5,
    )
    error_axis.annotate(
        "start",
        (tracker.sample_times[0], tracker.sample_errors[0]),
        xytext=(8, -14),
        textcoords="offset points",
        **_annotation_style("#2ca02c"),
    )
    if max_error_index is not None:
        error_axis.scatter(
            [tracker.sample_times[max_error_index]],
            [tracker.sample_errors[max_error_index]],
            color="#9467bd",
            s=42,
            zorder=5,
        )
        error_axis.annotate(
            "max |error|",
            (tracker.sample_times[max_error_index], tracker.sample_errors[max_error_index]),
            xytext=(8, -14),
            textcoords="offset points",
            **_annotation_style("#9467bd"),
        )
    error_axis.scatter(
        [tracker.sample_times[end_index]],
        [tracker.sample_errors[end_index]],
        color="#2ca02c",
        s=35,
        zorder=5,
    )
    error_axis.annotate(
        "end",
        (tracker.sample_times[end_index], tracker.sample_errors[end_index]),
        xytext=(-22, -14),
        textcoords="offset points",
        **_annotation_style("#2ca02c"),
    )

    title_parts = [f"{JOINT_NAME} tracking response"]  # 在标题里汇总本次试验最关键的跟踪指标。
    if rmse is not None:
        title_parts.append(f"RMSE={rmse:.6f}rad")
    fig.suptitle(" | ".join(title_parts))
    fig.tight_layout()

    output = Path(output_path)  # 统一按 Path 处理，便于自动创建父目录。
    output.parent.mkdir(parents=True, exist_ok=True)  # 若用户指定了不存在的目录，则自动创建。
    fig.savefig(output, dpi=160, bbox_inches="tight")  # 保存为 PNG 图像文件。
    plt.close(fig)  # 关闭 figure，避免脚本长期运行时累积资源占用。
    return str(output)  # 返回实际保存路径，便于主流程打印。


def save_comparison_plot(
    baseline: KpTrialResult,
    best: KpTrialResult,
    output_path: str,
) -> str:  # 生成 2x2 对比图：基线/最优 的位置曲线与误差曲线。
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(f"matplotlib is required for the 2x2 comparison plot: {exc}") from exc

    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex="col")  # 2x2 排列：左列基线，右列最佳。
    position_time_limits, position_value_limits = _comparison_position_limits(baseline, best)
    error_time_limits, error_value_limits = _comparison_error_limits(baseline, best)
    panels = [
        ("Baseline Position", baseline, axes[0][0], "position"),
        ("Best Kp Position", best, axes[0][1], "position"),
        ("Baseline Error", baseline, axes[1][0], "error"),
        ("Best Kp Error", best, axes[1][1], "error"),
    ]

    for title, trial, axis, mode in panels:
        tracker = trial.tracker
        reference_change_index = find_reference_change_index(
            tracker.sample_reference_positions,
            tracker.initial_reference_position,
        )
        max_error_index = find_max_abs_error_index(tracker.sample_errors)
        end_index = len(tracker.sample_times) - 1

        if mode == "position":
            position_offsets = _position_annotation_offsets(tracker, reference_change_index)
            axis.plot(
                tracker.sample_times,
                tracker.sample_reference_positions,
                label="reference",
                color="#d62728",
                linewidth=2.0,
            )
            axis.plot(
                tracker.sample_times,
                tracker.sample_positions,
                label="feedback",
                color="#1f77b4",
                linewidth=2.0,
            )
            axis.scatter(
                [tracker.sample_times[0]],
                [tracker.sample_reference_positions[0]],
                color="#d62728",
                s=35,
                zorder=5,
            )
            axis.annotate(
                "start ref",
                (tracker.sample_times[0], tracker.sample_reference_positions[0]),
                xytext=position_offsets["start_ref"],
                textcoords="offset points",
                **_annotation_style("#d62728"),
            )
            axis.scatter(
                [tracker.sample_times[0]],
                [tracker.sample_positions[0]],
                color="#1f77b4",
                s=35,
                zorder=5,
            )
            axis.annotate(
                "start fb",
                (tracker.sample_times[0], tracker.sample_positions[0]),
                xytext=position_offsets["start_fb"],
                textcoords="offset points",
                **_annotation_style("#1f77b4"),
            )
            if reference_change_index is not None:
                axis.scatter(
                    [tracker.sample_times[reference_change_index]],
                    [tracker.sample_reference_positions[reference_change_index]],
                    color="#ff7f0e",
                    s=42,
                    zorder=5,
                )
                axis.annotate(
                    "reference change",
                    (
                        tracker.sample_times[reference_change_index],
                        tracker.sample_reference_positions[reference_change_index],
                    ),
                    xytext=position_offsets["reference_change"],
                    textcoords="offset points",
                    **_annotation_style("#ff7f0e"),
                )
            axis.scatter(
                [tracker.sample_times[end_index]],
                [tracker.sample_positions[end_index]],
                color="#1f77b4",
                s=35,
                zorder=5,
            )
            axis.annotate(
                "end",
                (tracker.sample_times[end_index], tracker.sample_positions[end_index]),
                xytext=position_offsets["end"],
                textcoords="offset points",
                **_annotation_style("#1f77b4"),
            )
            axis.set_ylabel("Position (rad)")
            axis.set_xlim(*position_time_limits)
            axis.set_ylim(*position_value_limits)
        else:
            axis.plot(
                tracker.sample_times,
                tracker.sample_errors,
                label="tracking error",
                color="#2ca02c",
                linewidth=2.0,
            )
            axis.axhline(0.0, color="#666", linestyle="--", linewidth=1.2)
            axis.scatter(
                [tracker.sample_times[0]],
                [tracker.sample_errors[0]],
                color="#2ca02c",
                s=35,
                zorder=5,
            )
            axis.annotate(
                "start",
                (tracker.sample_times[0], tracker.sample_errors[0]),
                xytext=(8, -14),
                textcoords="offset points",
                **_annotation_style("#2ca02c"),
            )
            if max_error_index is not None:
                axis.scatter(
                    [tracker.sample_times[max_error_index]],
                    [tracker.sample_errors[max_error_index]],
                    color="#9467bd",
                    s=42,
                    zorder=5,
                )
                axis.annotate(
                    "max |error|",
                    (tracker.sample_times[max_error_index], tracker.sample_errors[max_error_index]),
                    xytext=(8, -14),
                    textcoords="offset points",
                    **_annotation_style("#9467bd"),
                )
            axis.scatter(
                [tracker.sample_times[end_index]],
                [tracker.sample_errors[end_index]],
                color="#2ca02c",
                s=35,
                zorder=5,
            )
            axis.annotate(
                "end",
                (tracker.sample_times[end_index], tracker.sample_errors[end_index]),
                xytext=(-22, -14),
                textcoords="offset points",
                **_annotation_style("#2ca02c"),
            )
            axis.set_ylabel("Error (rad)")
            axis.set_xlabel("Time since command (s)")
            axis.set_xlim(*error_time_limits)
            axis.set_ylim(*error_value_limits)

        axis.set_title(
            f"{title}\nKp={trial.kp:.4f}, RMSE={trial.metrics.rmse:.6f}, Cost={trial.metrics.cost:.6f}"
        )
        axis.minorticks_on()
        axis.grid(True, which="major", linestyle="--", alpha=0.45)
        axis.grid(True, which="minor", linestyle=":", alpha=0.2)
        axis.tick_params(axis="both", labelsize=10)
        axis.legend(loc="best")

    fig.suptitle(f"{JOINT_NAME} Kp Comparison: Baseline vs Best", fontsize=16)
    fig.tight_layout()

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return str(output)


def build_command(req: MotionRequest) -> str:  # 根据 MotionRequest 拼接 ExecuteCommand 所需的命令字符串。
    parts = [
        "group=r_arm",
        "type=joints",
        f"joints={JOINT_NAME}:{req.position}",
        f"vel={req.vel}",
        f"acc={req.acc}",
        f"pipeline={req.pipeline}",
        f"planner_id={req.planner_id}",
        f"exec={1 if req.exec_motion else 0}",
    ]
    return " ".join(parts)


def parse_args() -> argparse.Namespace:  # 定义并解析命令行参数。
    parser = argparse.ArgumentParser(  # 创建参数解析器，并设置帮助说明。
        description="Control right_arm_1_joint via /cli_controller/execute_command."
    )
    parser.add_argument(  # 添加位置参数 `position`。
        "position",
        type=float,
        help="Target joint position for right_arm_1_joint, in radians.",
    )
    parser.add_argument(  # 添加可选参数 `--service`。
        "--service",
        default="/cli_controller/execute_command",
        help="ROS 2 service name. Default: /cli_controller/execute_command",
    )
    parser.add_argument(  # 添加可选参数 `--vel`。
        "--vel",
        type=float,
        default=0.3,
        help="Planning velocity scaling. Default: 0.3",
    )
    parser.add_argument(  # 添加可选参数 `--acc`。
        "--acc",
        type=float,
        default=0.3,
        help="Planning acceleration scaling. Default: 0.3",
    )
    parser.add_argument(  # 添加可选参数 `--pipeline`。
        "--pipeline",
        default="ompl",
        help="Planning pipeline. Default: ompl",
    )
    parser.add_argument(  # 添加可选参数 `--planner-id`。
        "--planner-id",
        default="RRTConnectkConfigDefault",
        help="Planner id. Default: RRTConnectkConfigDefault",
    )
    parser.add_argument(  # 添加可选参数 `--timeout`。
        "--timeout",
        type=float,
        default=0.0,
        help="Service request timeout field, in seconds. Default: 0.0",
    )
    parser.add_argument(  # 添加可选参数 `--spin-timeout`。
        "--spin-timeout",
        type=float,
        default=10.0,
        help="Local wait timeout for the service response, in seconds. Default: 10.0",
    )
    parser.add_argument(  # 添加布尔开关 `--no-wait`。
        "--no-wait",
        action="store_true",
        help="Do not wait for motion result from the service.",
    )
    parser.add_argument(  # 添加布尔开关 `--plan-only`。
        "--plan-only",
        action="store_true",
        help="Set exec=0 to plan without execution.",
    )
    parser.add_argument(  # 添加可选参数 `--state-topic`，用于指定控制器状态话题。
        "--state-topic",
        default="/r_arm_controller/controller_state",
        help="Controller state topic used to plot reference and feedback. Default: /r_arm_controller/controller_state",
    )
    parser.add_argument(  # 添加可选参数 `--state-timeout`，用于限制等待初始控制器状态的时间。
        "--state-timeout",
        type=float,
        default=2.0,
        help="Wait timeout for the initial controller state, in seconds. Default: 2.0",
    )
    parser.add_argument(  # 添加可选参数 `--observe-window`，用于指定命令发出后继续采样多久。
        "--observe-window",
        type=float,
        default=3.0,
        help="Observation window for tracking error collection after the command is sent, in seconds. Default: 3.0",
    )
    parser.add_argument(  # 添加可选参数 `--plot-output`，用于指定响应曲线图保存路径。
        "--plot-output",
        default=None,
        help="PNG output path for the response plot. Default: auto-generated in the script directory",
    )
    parser.add_argument(  # 添加布尔开关 `--no-plot`，允许用户只测量不画图。
        "--no-plot",
        action="store_true",
        help="Do not generate a response plot image.",
    )
    parser.add_argument(  # 添加布尔开关 `--auto-tune-kp`，开启位置环 Kp 自动搜索。
        "--auto-tune-kp",
        action="store_true",
        help="Automatically search for a better position-loop Kp before the final run.",
    )
    parser.add_argument(  # 添加可选参数 `--kp-alias`，用于指定 EtherCAT alias。
        "--kp-alias",
        type=int,
        default=DEFAULT_KP_ALIAS,
        help=f"EtherCAT alias of the joint drive. Default: {DEFAULT_KP_ALIAS}",
    )
    parser.add_argument(  # 添加可选参数 `--kp-min`，不指定时自动在当前 Kp 下方浮动 kp_range。
        "--kp-min",
        type=float,
        default=None,
        help="Lower bound of Kp search interval. Default: current_kp - kp_range",
    )
    parser.add_argument(  # 添加可选参数 `--kp-max`，不指定时自动在当前 Kp 上方浮动 kp_range。
        "--kp-max",
        type=float,
        default=None,
        help="Upper bound of Kp search interval. Default: current_kp + kp_range",
    )
    parser.add_argument(  # 添加可选参数 `--kp-range`，在当前 Kp 上下各浮动此值构成默认搜索区间。
        "--kp-range",
        type=float,
        default=DEFAULT_KP_RANGE,
        help=f"Kp search range offset from current Kp. Default: {DEFAULT_KP_RANGE}",
    )
    parser.add_argument(  # 添加可选参数 `--kp-index`，用于覆盖 SDO index。
        "--kp-index",
        default=DEFAULT_KP_INDEX,
        help=f"SDO index of position-loop Kp. Default: {DEFAULT_KP_INDEX}",
    )
    parser.add_argument(  # 添加可选参数 `--kp-subindex`，用于覆盖 SDO subindex。
        "--kp-subindex",
        type=int,
        default=DEFAULT_KP_SUBINDEX,
        help=f"SDO subindex of position-loop Kp. Default: {DEFAULT_KP_SUBINDEX}",
    )
    parser.add_argument(  # 添加可选参数 `--kp-iterations`，用于限制搜索轮数。
        "--kp-iterations",
        type=int,
        default=4,
        help="Maximum number of Kp search iterations. Default: 4",
    )
    return parser.parse_args()


def main() -> int:  # 主函数，返回进程退出码。
    args = parse_args()

    try:  # 尝试导入 ROS 2 运行和关节反馈测量所需的模块。
        import rclpy  # ROS 2 Python 客户端库。
        from control_msgs.msg import JointTrajectoryControllerState  # 导入控制器状态消息，用于读取 reference 和 feedback。
        from rclpy.node import Node  # ROS 2 节点基类。
        from robot_controller.srv import ExecuteCommand  # 导入命令执行服务类型。
    except ImportError as exc:  # 如果导入失败，通常说明环境还没 source。
        print(  # 打印明确的错误提示，指导用户先 source 环境。
            "Failed to import ROS 2 dependencies. "
            "Source ROS and raybot_core_ws before running this script.",
            file=sys.stderr,
        )
        print(  # 给出当前机器上可直接执行的环境初始化命令。
            "Example:",
            file=sys.stderr,
        )
        print(
            "  source /opt/ros/humble/setup.bash",
            file=sys.stderr,
        )
        print(
            "  source /home/raybot/raybot_core_ws/install/setup.bash",
            file=sys.stderr,
        )
        print(f"Import error: {exc}", file=sys.stderr)  # 补充具体导入异常内容。
        return 1  # 返回非零退出码，表示脚本执行失败。

    class ExecuteCommandClient(Node):  # 定义一个最小 ROS 2 客户端节点，用于调用 ExecuteCommand 服务。
        def __init__(self, service: str, state_topic: str) -> None:  # 初始化节点、服务客户端和控制器状态订阅。
            super().__init__("move_right_arm_1_joint_client")
            self._client = self.create_client(ExecuteCommand, service)
            self._controller_state_sub = self.create_subscription(
                JointTrajectoryControllerState,
                state_topic,
                self._controller_state_callback,
                10,
            )
            self._latest_reference_position: Optional[float] = None  # 缓存控制器 reference 中的目标值。
            self._latest_feedback_position: Optional[float] = None  # 缓存控制器 feedback 中的实际值。
            self._joint_index: Optional[int] = None  # 缓存 right_arm_1_joint 在 controller_state 数组中的索引。
            self._response_tracker: Optional[ResponseTracker] = None  # 缓存当前这次动作的响应测量器。

        def _controller_state_callback(self, msg: JointTrajectoryControllerState) -> None:  # 从 controller_state 中提取 reference 和 feedback。
            try:  # 提取目标值和实际值；当前右臂单关节脚本只关心数组中的第一个量。
                if self._joint_index is None:  # 首次收到消息时，先根据 joint_names 定位目标关节索引。
                    self._joint_index = msg.joint_names.index(JOINT_NAME)
                reference_position = extract_position(msg.reference.positions, self._joint_index)
                feedback_position = extract_position(msg.feedback.positions, self._joint_index)
            except (AttributeError, ValueError):  # 消息字段不完整时直接忽略本帧。
                return

            self._latest_reference_position = reference_position  # 更新 reference 缓存，供响应曲线和目标值计算使用。
            self._latest_feedback_position = feedback_position  # 更新 feedback 缓存，供动作前后查询。

            if self._response_tracker is not None:  # 若当前正在测量响应，则同步更新阈值和峰值。
                self._response_tracker.update(reference_position, feedback_position, time.monotonic())  # 用 controller_state.reference 作为目标、feedback 作为实际响应。

        def wait_for_controller_state(self, timeout_sec: float) -> tuple[float, float]:  # 等待首次拿到控制器 reference 和 feedback。
            deadline = time.monotonic() + timeout_sec  # 用单调时钟构造截止时间，避免系统时钟跳变。
            while rclpy.ok() and time.monotonic() < deadline:  # 在超时前持续自旋处理 controller_state 回调。
                if (
                    self._latest_reference_position is not None
                    and self._latest_feedback_position is not None
                ):  # 一旦收到有效 reference 和 feedback 就立即返回。
                    return self._latest_reference_position, self._latest_feedback_position
                rclpy.spin_once(self, timeout_sec=0.1)  # 短周期自旋，兼顾响应和 CPU 占用。

            if (
                self._latest_reference_position is not None
                and self._latest_feedback_position is not None
            ):  # 边界情况下允许在最后一次检查时成功返回。
                return self._latest_reference_position, self._latest_feedback_position

            raise TimeoutError(  # 超时仍无反馈时抛错，提示用户检查状态话题。
                f"Timed out waiting for controller state after {timeout_sec}s"
            )

        def start_response_tracking(
            self,
            initial_reference: float,
            initial_feedback: float,
            target_position: float,
        ) -> ResponseTracker:  # 基于 controller_state 中的 reference 和 feedback 初始化一次响应测量。
            tracker = ResponseTracker(
                initial_position=initial_feedback,
                initial_reference_position=initial_reference,
                target_position=target_position,
            )
            tracker.last_position = initial_feedback  # 先记录初始实际值，命令发出前不计入响应曲线时间轴。
            tracker.last_reference_position = initial_reference  # 先记录初始 reference，便于后续排查参考轨迹是否切换。
            self._response_tracker = tracker if tracker.has_motion() else None  # 只有真运动时才需要后续持续跟踪。
            return tracker

        def observe_tracking_window(self, window_sec: float) -> None:  # 在命令发出后继续观测一段时间，用于采样跟踪误差。
            tracker = self._response_tracker  # 取出当前动作的响应测量器。
            if tracker is None:  # 无动作或未启用跟踪时，不需要额外观测。
                return

            deadline = time.monotonic() + window_sec  # 从当前时刻开始继续采样指定时间窗口。
            while rclpy.ok() and time.monotonic() < deadline:  # 在超时前持续处理 controller_state 回调。
                rclpy.spin_once(self, timeout_sec=0.1)  # 继续消费状态反馈。

        def call(self, req: MotionRequest):  # 封装一次完整的“先采状态、再发命令、再返回测量上下文”的流程。
            if not self._client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("Service not available")

            initial_reference, initial_feedback = self.wait_for_controller_state(req.state_timeout)
            tracker = self.start_response_tracking(initial_reference, initial_feedback, req.position)

            request = ExecuteCommand.Request()
            request.command = build_command(req)
            request.wait_for_result = not req.no_wait
            request.timeout = req.timeout

            tracker.set_command_time(time.monotonic())
            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=req.spin_timeout)

            if not future.done():
                raise TimeoutError(
                    f"Timed out waiting for service response after {req.spin_timeout}s"
                )

            response = future.result()  # 取出服务响应结果。
            if response is None:  # 理论上不应为空，这里做防御性检查。
                raise RuntimeError("Service returned no response")
            return request.command, response, initial_feedback, tracker  # 返回命令、服务响应和本次测量上下文。

    def finalize_trial(
        tracker: ResponseTracker,
        response,
        observe_window: float,
    ) -> TrackingMetrics:  # 基于一次动作执行后的观测窗口，计算跟踪评价指标。
        if not response.accepted:
            raise RuntimeError("command not accepted")
        if response.result_received and not response.result.executed:
            raise RuntimeError("motion not executed")

        node.observe_tracking_window(observe_window)
        metrics = compute_tracking_metrics(tracker)  # 基于整个观测窗口内的采样计算 RMSE/相位滞后/幅值衰减等指标。
        return metrics

    def print_metrics(metrics: TrackingMetrics, prefix: str = "") -> None:  # 统一打印评价指标，方便单次运行和自动调参都复用。
        header = f"{prefix}" if prefix else ""
        print(f"{header}rmse: {metrics.rmse:.6f} rad")
        print(f"{header}phase_lag: {metrics.phase_lag_deg:.6f} deg")
        print(f"{header}amplitude_ratio: {metrics.amplitude_ratio:.6f}")
        print(f"{header}overshoot: {metrics.overshoot_rad:.6f} rad")
        print(f"{header}rmse_term: {metrics.rmse_term:.6f}")
        print(f"{header}phase_lag_term: {metrics.phase_lag_term:.6f}")
        print(f"{header}amplitude_ratio_term: {metrics.amplitude_ratio_term:.6f}")
        print(f"{header}rmse_penalty: {metrics.rmse_penalty:.6f}")
        print(f"{header}phase_lag_penalty: {metrics.phase_lag_penalty:.6f}")
        print(f"{header}amplitude_ratio_penalty: {metrics.amplitude_ratio_penalty:.6f}")
        print(f"{header}overshoot_penalty: {metrics.overshoot_penalty:.6f}")
        print(f"{header}cost: {metrics.cost:.6f}")
        print(f"{header}meets_target: {metrics.meets_target}")
        if metrics.note:
            print(f"{header}note: {metrics.note}")

    def run_single_motion_trial(req: MotionRequest) -> tuple[str, object, float, ResponseTracker, TrackingMetrics]:  # 执行一次运动并返回完整测量结果。
        command, response, initial_position, tracker = node.call(req)
        metrics = finalize_trial(tracker, response, req.observe_window)
        return command, response, initial_position, tracker, metrics

    def move_joint_to(position: float, base_req: MotionRequest) -> None:  # 用当前脚本同一路径把关节移动到指定位置，便于候选试验前回到统一起点。
        reset_req = replace(base_req, position=position)
        _, _, _, _, _ = run_single_motion_trial(reset_req)

    def evaluate_kp_candidate(kp: float, reset_position: float, req: MotionRequest) -> KpTrialResult:  # 对某个候选 Kp 执行一次闭环试验，并计算总代价。
        move_joint_to(reset_position, req)
        write_kp(args.kp_alias, args.kp_index, args.kp_subindex, kp)
        _, _, _, tracker, metrics = run_single_motion_trial(req)
        return KpTrialResult(kp=kp, tracker=tracker, metrics=metrics)

    def auto_tune_kp(req: MotionRequest) -> AutoTuneResult:  # 在给定区间内自动搜索一个代价更低的位置环 Kp。
        if args.kp_iterations < 1:
            raise RuntimeError("--kp-iterations must be at least 1")

        original_kp = read_kp(args.kp_alias, args.kp_index, args.kp_subindex)
        kp_range = args.kp_range
        low = max(0.0, original_kp - kp_range) if args.kp_min is None else args.kp_min
        high = original_kp + kp_range if args.kp_max is None else args.kp_max
        if low >= high:
            raise RuntimeError(f"Kp search interval is empty or inverted: [{low:.6f}, {high:.6f}]")
        print(f"kp_original: {original_kp:.6f}, search_interval: [{low:.6f}, {high:.6f}]")
        _, reset_position = node.wait_for_controller_state(req.state_timeout)
        baseline_result = evaluate_kp_candidate(original_kp, reset_position, req)
        print_metrics(baseline_result.metrics, prefix=f"kp={original_kp:.6f} ")

        history: list[KpTrialResult] = []
        best: Optional[KpTrialResult] = baseline_result
        evaluated_costs: dict[int, float] = {}
        evaluated_costs[round(original_kp, 6)] = baseline_result.metrics.cost

        try:
            for iteration in range(args.kp_iterations):
                if high - low < KP_INTERVAL_TOLERANCE:
                    print(f"kp_search[{iteration + 1}/{args.kp_iterations}]: interval converged ({low:.6f}, {high:.6f})")
                    break

                left = low + (high - low) / 3.0
                right = high - (high - low) / 3.0
                print(f"kp_search[{iteration + 1}/{args.kp_iterations}]: interval=({low:.6f}, {high:.6f})")

                left_key = round(left, 6)
                if left_key not in evaluated_costs:
                    left_result = evaluate_kp_candidate(left, reset_position, req)
                    print_metrics(left_result.metrics, prefix=f"kp={left:.6f} ")
                    history.append(left_result)
                    evaluated_costs[left_key] = left_result.metrics.cost
                    if best is None or left_result.metrics.cost < best.metrics.cost:
                        best = left_result

                right_key = round(right, 6)
                if right_key not in evaluated_costs:
                    right_result = evaluate_kp_candidate(right, reset_position, req)
                    print_metrics(right_result.metrics, prefix=f"kp={right:.6f} ")
                    history.append(right_result)
                    evaluated_costs[right_key] = right_result.metrics.cost
                    if best is None or right_result.metrics.cost < best.metrics.cost:
                        best = right_result

                if evaluated_costs[left_key] <= evaluated_costs[right_key]:
                    high = right
                else:
                    low = left

                if best.metrics.meets_target and iteration + 1 >= KP_MIN_ITERATIONS:
                    break

            if best is None:
                raise RuntimeError("Kp auto-tuning produced no valid trials")

            write_kp(args.kp_alias, args.kp_index, args.kp_subindex, best.kp)
            print(f"kp_best: {best.kp:.6f}")
            return AutoTuneResult(
                best_kp=best.kp,
                history=history,
                original_kp=original_kp,
                baseline_result=baseline_result,
                reset_position=reset_position,
            )
        except Exception:
            write_kp(args.kp_alias, args.kp_index, args.kp_subindex, original_kp)
            raise

    rclpy.init()
    node = ExecuteCommandClient(args.service, args.state_topic)
    try:
        original_kp: Optional[float] = None
        history: list[KpTrialResult] = []
        baseline_result: Optional[KpTrialResult] = None
        reset_position: Optional[float] = None
        kp_trials_csv_path: Optional[str] = None
        should_save_single_plot = not args.no_plot
        base_req = MotionRequest(
            position=args.position,
            vel=args.vel,
            acc=args.acc,
            pipeline=args.pipeline,
            planner_id=args.planner_id,
            exec_motion=not args.plan_only,
            no_wait=args.no_wait,
            timeout=args.timeout,
            spin_timeout=args.spin_timeout,
            state_timeout=args.state_timeout,
            observe_window=args.observe_window,
        )

        if args.auto_tune_kp:
            result = auto_tune_kp(base_req)
            best_kp = result.best_kp
            history = result.history
            original_kp = result.original_kp
            baseline_result = result.baseline_result
            reset_position = result.reset_position
            print(f"kp_selected: {best_kp:.6f}")
            kp_trials_csv_path = save_kp_trials_csv(
                baseline_result,
                history,
                best_kp,
                default_kp_trials_csv_path(),
            )
            should_save_single_plot = False
            move_joint_to(reset_position, base_req)

        command, response, initial_position, tracker, metrics = run_single_motion_trial(base_req)
    except Exception as exc:
        print(f"Failed to move {JOINT_NAME}: {exc}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"command: {command}")  # 打印实际下发的命令字符串，便于排查问题。
    print(f"accepted: {response.accepted}")  # 打印服务端是否受理了这条命令。
    print(f"success: {response.success}")  # 打印服务端报告的整体成功状态。
    print(f"command_id: {response.command_id}")  # 打印命令 ID，便于关联日志。
    print(f"message: {response.message}")  # 打印服务端返回的说明信息。
    print(f"initial_position: {initial_position}")  # 打印发命令前读取到的关节初始角度。
    print(f"target_position: {base_req.position}")
    print(f"delta_position: {tracker.target_position - tracker.initial_position}")  # 打印本次动作总位移，便于结合响应指标分析。

    if response.result_received:  # 如果本次请求要求等待结果，并且确实收到了结果。
        print(f"moveit_code: {response.result.moveit_code}")  # 打印 MoveIt 返回码。
        print(f"executed: {response.result.executed}")  # 打印是否真的执行了，而不只是规划。
        print(f"result_message: {response.result.message}")  # 打印更详细的执行结果描述。

    print_metrics(metrics)  # 打印最终试验的综合跟踪指标。
    if args.auto_tune_kp:
        print(f"kp_target_satisfied: {metrics.meets_target}")
        print(f"kp_trials: {len(history)}")
        if kp_trials_csv_path is not None:
            print(f"kp_trials_csv: {kp_trials_csv_path}")

    if should_save_single_plot and tracker.command_time is not None:
        plot_output = args.plot_output or default_plot_path(base_req.position)
        try:
            actual_plot_path = save_response_plot(tracker, plot_output, metrics.rmse)
            print(f"response_plot: {actual_plot_path}")  # 打印图像路径，便于用户直接打开查看。
        except Exception as exc:
            print(f"response_plot: unavailable ({exc})")  # 绘图失败时不影响主流程，只输出原因。

    if args.auto_tune_kp and baseline_result is not None and not args.no_plot:
        plot_base_path = Path(args.plot_output or default_plot_path(base_req.position))
        comparison_output = plot_base_path.with_name(
            f"{plot_base_path.stem}_comparison.png"
        )
        try:
            comparison_path = save_comparison_plot(
                baseline_result,
                KpTrialResult(
                    kp=best_kp,
                    tracker=tracker,
                    metrics=metrics,
                ),
                str(comparison_output),
            )
            print(f"comparison_plot: {comparison_path}")
        except Exception as exc:
            print(f"comparison_plot: unavailable ({exc})")

    if args.auto_tune_kp and original_kp is not None:  # 自动调参模式下，最终仍恢复原始 Kp，避免脚本隐式改变现场参数。
        try:
            write_kp(args.kp_alias, args.kp_index, args.kp_subindex, original_kp)
            print(f"kp_restored: {original_kp:.6f}")
        except Exception as exc:
            print(f"kp_restore_failed: {exc}", file=sys.stderr)

    node.destroy_node()  # 正常结束前销毁 ROS 2 节点。
    rclpy.shutdown()  # 关闭 ROS 2 Python 运行时。
    return 0 if response.success else 2  # 成功返回 0；服务有响应但执行失败时返回 2。


if __name__ == "__main__":  # 只有直接运行本文件时才进入这里；被 import 时不会自动执行。
    sys.exit(main())  # 执行主函数，并把返回值作为进程退出码。
