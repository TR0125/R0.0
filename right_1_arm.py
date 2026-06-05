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


@dataclass(frozen=True)
class ArmJointSpec:  # 保存单个手臂关节的控制器和 EtherCAT 映射。
    joint_name: str
    group: str
    state_topic: str
    kp_alias: int


LEFT_ARM_JOINTS = [f"left_arm_{index}_joint" for index in range(1, 8)]
RIGHT_ARM_JOINTS = [f"right_arm_{index}_joint" for index in range(1, 8)]
ALL_ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
DEFAULT_JOINT_NAME = "right_arm_1_joint"
ARM_JOINT_SPECS = {
    **{
        joint_name: ArmJointSpec(
            joint_name=joint_name,
            group="l_arm",
            state_topic="/l_arm_controller/controller_state",
            kp_alias=1102 + index,
        )
        for index, joint_name in enumerate(LEFT_ARM_JOINTS)
    },
    **{
        joint_name: ArmJointSpec(
            joint_name=joint_name,
            group="r_arm",
            state_topic="/r_arm_controller/controller_state",
            kp_alias=1109 + index,
        )
        for index, joint_name in enumerate(RIGHT_ARM_JOINTS)
    },
}
JOINT_NAME = DEFAULT_JOINT_NAME  # 当前脚本控制并监测的目标关节名，会在 parse_args 后按参数更新。
COMMAND_GROUP = ARM_JOINT_SPECS[DEFAULT_JOINT_NAME].group  # ExecuteCommand 使用的 MoveIt group。
POSITION_EPSILON = 1e-6  # 用于判断“目标已达到/几乎无位移”的最小阈值。
DEFAULT_KP_ALIAS = ARM_JOINT_SPECS[DEFAULT_JOINT_NAME].kp_alias  # 默认关节的 EtherCAT alias。
DEFAULT_KP_INDEX = "0x2006"  # 位置环 P 参数的 SDO index。
DEFAULT_KP_SUBINDEX = 0  # 位置环 P 参数的 SDO subindex。
FLOAT_PATTERN = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")  # 用于从命令输出中提取浮点数字。
DEFAULT_AUTO_TUNE_ACTION_COUNT = 3  # 自动调参默认使用 3 个测试动作。
RMSE_TARGET_DEG = 0.01  # 搜索代价中的 RMSE 参考目标，按角度制指定为 0.01 度。
RMSE_TARGET_RAD = math.radians(RMSE_TARGET_DEG)  # 搜索代价内部统一换算成弧度参与归一化。
PHASE_LAG_TARGET_DEG = 2.0  # 搜索代价中的相位滞后参考目标，按角度制指定为 2 度。
PHASE_LAG_EVAL_FREQUENCY_HZ = 1.0  # 用于把时间延迟换算成相位角的等效频率，默认按 1 Hz 解释。
AMPLITUDE_RATIO_TARGET = 1.0  # 幅值比理想值为 1.0。
AMPLITUDE_RATIO_MIN = 0.95  # 搜索和阶段过渡判定使用的最低幅值比要求。
RMSE_EXCESS_PENALTY_WEIGHT = 100.0  # RMSE 超过目标后施加的大罚分，保证搜索优先压低 RMSE。
PHASE_LAG_EXCESS_PENALTY_WEIGHT = 50.0  # 相位滞后超过目标角度后施加的附加罚分。
AMPLITUDE_RATIO_DEFICIT_PENALTY_WEIGHT = 80.0  # 幅值比低于 0.95 后施加的附加罚分。
OVERSHOOT_HARD_PENALTY = 1000.0  # 超过阈值后施加超调硬惩罚，使“无显著超调”成为近似硬约束。
OVERSHOOT_STRICT_MAX_RAD = 1e-4  # 最终严格验收允许的最大超调。
OVERSHOOT_PENALTY_THRESHOLD = 5e-5  # 搜索代价中开始施加超调硬惩罚的阈值。
OVERSHOOT_EXPAND_MAX_RAD = 1.5e-4  # 允许进入下一阶段时的超调上限。
STRICT_RMSE_MAX_DEG = 0.02  # 最终严格验收使用的 RMSE 阈值，按角度制指定。
STRICT_RMSE_MAX_RAD = math.radians(STRICT_RMSE_MAX_DEG)  # 最终严格验收内部统一换算成弧度。
STRICT_PHASE_LAG_MAX_DEG = 20.0  # 最终严格验收使用的相位滞后阈值。
STRICT_AMPLITUDE_RATIO_MIN = 0.98  # 最终严格验收使用的最小幅值比。
RMSE_COST_WEIGHT = 1.0  # RMSE 在基础代价中的权重。
PHASE_LAG_COST_WEIGHT = 0.8  # 相位滞后在基础代价中的权重，增强动态性能影响。
AMPLITUDE_RATIO_COST_WEIGHT = 0.25  # 幅值比在基础代价中的权重。
SMOOTHNESS_COST_WEIGHT = 0.5  # 尾部平滑度在搜索代价中的权重，抑制高频抖动。
UNSTABLE_PENALTY = 2000.0  # 一旦判定尾部不稳定，额外施加大罚分，避免把抖动点选为最优。
KP_INTERVAL_TOLERANCE = 0.001  # Kp 搜索区间宽度小于此值时，可认为搜索范围已经足够收敛。
KP_MIN_ITERATIONS = 2  # 即使已达标，也至少跑这么多轮再停止。
DEFAULT_KP_RELATIVE_RANGE_RATIO = 0.1  # 未显式给出区间时，初始阶段默认围绕当前 Kp 做 ±10% 搜索。
DEFAULT_KP_HARD_MAX_FACTOR = 2.5  # 自动调参时，hard_max 默认为 original_kp 的此倍数。
DEFAULT_KP_HARD_MAX_LIMIT = 1.0  # 自动调参绝对硬上限，强制不超过 1.0。
KP_BAYES_GRID_SIZE = 81  # 贝叶斯优化候选网格密度；一维问题下用稠密网格即可稳定选点。
KP_BAYES_INITIAL_SAMPLE_COUNT = 3  # 每个阶段先采 low/mid/high 三个种子点，避免 seed 吃掉过多预算。
KP_BAYES_NOISE_VARIANCE = 1e-6  # 高斯过程观测噪声项，避免协方差矩阵奇异。
KP_BAYES_JITTER = 1e-9  # 数值稳定抖动项，避免 Cholesky 分解因舍入误差失败。
KP_BAYES_EI_TOLERANCE = 1e-3  # 归一化后 EI 低于此值时，认为继续搜索价值很低。
KP_DUPLICATE_TOLERANCE = 1e-4  # 候选 Kp 若与历史点过近，则视为重复点，不再重复评估。
DEFAULT_KP_REPEATS = 2  # 自动调参时同一 Kp 默认重复试验次数，用均值降低现场噪声影响。
SEARCH_COST_STD_WEIGHT = 1.0  # 最终候选比较时使用 mean(search_cost) + lambda * std(search_cost) 中的 lambda。
STABLE_TAIL_RATIO = 0.25  # 取末尾 25% 样本评估尾部振荡和抖动。
STABLE_TAIL_STD_MAX_RAD = 1e-4  # 尾部标准差上限，超过则认为尾部仍在抖动。
STABLE_TAIL_PEAK_TO_PEAK_MAX_RAD = 3e-4  # 尾部峰峰值上限，超过则认为尾部仍在振荡。
STABLE_TAIL_DIFF_RMS_MAX_RAD = 1e-4  # 尾部相邻样本差分 RMS 上限，超过则认为曲线不平滑。
SMALL_STEP_RATIO = 0.2  # 小步进测试相对主动作位移的比例。
SMALL_STEP_MIN_RAD = 0.05  # 小步进测试的最小位移幅度。
SMALL_STEP_MAX_RAD = 0.15  # 小步进测试的最大位移幅度。
AUTO_TUNE_MAX_STAGES = 3  # 自动调参最多串行执行 3 个阶段：趋势搜索、局部精调、最终确认。
AUTO_TUNE_STAGE_ITERATIONS = (4, 6, 8)  # 三阶段默认预算分配；用户指定更大迭代数时按比例放大。
AUTO_TUNE_STAGE_HALF_WIDTHS = (0.10, 0.08, 0.03)  # 后续阶段围绕上一阶段最优点自动收窄搜索区间的半宽度。
AUTO_TUNE_MIN_ITERATIONS = AUTO_TUNE_MAX_STAGES * KP_BAYES_INITIAL_SAMPLE_COUNT  # 若要真正完成 3 个阶段，至少要给到每阶段 low/mid/high 三个点评估预算。


def sanitize_identifier(value: str) -> str:  # 将关节名转成可用于 ROS node name / 文件名的安全字符串。
    return re.sub(r"[^A-Za-z0-9_]+", "_", value)


def configure_joint_context(args: argparse.Namespace) -> None:  # 根据 --joint-name 解析本次运行的关节上下文。
    global JOINT_NAME, COMMAND_GROUP
    spec = ARM_JOINT_SPECS.get(args.joint_name)
    if spec is None:
        missing = []
        if args.group is None:
            missing.append("--group")
        if args.state_topic is None:
            missing.append("--state-topic")
        if args.auto_tune_kp and args.kp_alias is None:
            missing.append("--kp-alias")
        if missing:
            raise ValueError(
                f"Unknown joint {args.joint_name!r}; provide {'/'.join(missing)} explicitly"
            )
        resolved_group = args.group
        resolved_state_topic = args.state_topic
        resolved_kp_alias = args.kp_alias
    else:
        resolved_group = args.group or spec.group
        resolved_state_topic = args.state_topic or spec.state_topic
        resolved_kp_alias = args.kp_alias if args.kp_alias is not None else spec.kp_alias

    JOINT_NAME = args.joint_name
    COMMAND_GROUP = resolved_group
    args.group = resolved_group
    args.state_topic = resolved_state_topic
    args.kp_alias = resolved_kp_alias


def parse_action_positions(raw_value: str) -> list[float]:  # 解析 "0.1,0.2,0.3" 形式的非交互测试动作。
    values = [part.strip() for part in raw_value.split(",") if part.strip()]
    if not values:
        raise ValueError("--auto-tune-actions must contain at least one numeric position")
    try:
        return [float(value) for value in values]
    except ValueError as exc:
        raise ValueError(f"Invalid --auto-tune-actions value {raw_value!r}") from exc


def parse_joint_position_map(raw_value: str) -> dict[str, float]:  # 解析 "joint_a:0.1,joint_b:-0.2" 形式的关节目标映射。
    result: dict[str, float] = {}
    items = [part.strip() for part in raw_value.split(",") if part.strip()]
    if not items:
        raise ValueError("--setup-joints must contain at least one joint:position pair")

    for item in items:
        joint_name, separator, position_text = item.partition(":")
        joint_name = joint_name.strip()
        position_text = position_text.strip()
        if not separator or not joint_name or not position_text:
            raise ValueError(f"Invalid --setup-joints item {item!r}; expected joint_name:position")
        if joint_name in result:
            raise ValueError(f"Duplicate setup joint in --setup-joints: {joint_name}")
        if joint_name not in ALL_ARM_JOINTS:
            raise ValueError(f"Unknown setup joint in --setup-joints: {joint_name}")
        try:
            result[joint_name] = float(position_text)
        except ValueError as exc:
            raise ValueError(f"Invalid setup position for {joint_name}: {position_text!r}") from exc

    return result


def arm_joints_for(joint_name: str) -> list[str]:  # 返回目标关节所属手臂的 7 个关节，用于校验 setup 姿态。
    if joint_name in LEFT_ARM_JOINTS:
        return LEFT_ARM_JOINTS
    if joint_name in RIGHT_ARM_JOINTS:
        return RIGHT_ARM_JOINTS
    raise ValueError(f"Setup joints are only supported for known arm joints: {joint_name}")


def validate_setup_joints(joint_name: str, setup: dict[str, float]) -> None:  # setup 必须是同臂除目标外的完整 6 个辅助关节。
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
            f"--setup-joints for {joint_name} must define exactly the other 6 joints in the same arm; "
            + "; ".join(details)
        )


def ordered_joint_positions(joint_positions: dict[str, float]) -> list[tuple[str, float]]:  # 按左右臂自然顺序输出命令，便于 dry-run 排查。
    ordered_names = [joint for joint in ALL_ARM_JOINTS if joint in joint_positions]
    ordered_names.extend(joint for joint in joint_positions if joint not in ordered_names)
    return [(joint_name, joint_positions[joint_name]) for joint_name in ordered_names]


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
    tail_std_rad: float  # 末尾稳态窗口的标准差，用于判断尾部抖动。
    tail_peak_to_peak_rad: float  # 末尾稳态窗口的峰峰值，用于判断尾部振荡。
    tail_diff_rms_rad: float  # 末尾稳态窗口相邻样本差分 RMS，用于衡量平滑度。
    rmse_term: float  # RMSE 按目标值 0.01 度归一化后的代价值分量。
    phase_lag_term: float  # 相位滞后按目标角度归一化后的代价值分量。
    amplitude_ratio_term: float  # 幅值比相对理想值的代价值分量。
    smoothness_term: float  # 基于尾部抖动和平滑度的代价值分量。
    rmse_penalty: float  # RMSE 超目标后的附加罚分。
    phase_lag_penalty: float  # 相位滞后超目标角度后的附加罚分。
    amplitude_ratio_penalty: float  # 幅值比低于 0.95 后的附加罚分。
    overshoot_penalty: float  # 出现超调后的硬惩罚。
    search_cost: float  # 搜索阶段使用的总代价，供 BO 和候选比较使用。
    strict_ok: bool  # 是否满足最终严格验收标准。
    expand_safe: bool  # 是否满足进入下一阶段或作为稳定回退候选的宽松安全标准。
    stable_ok: bool  # 是否满足尾部稳定和平滑性硬门槛。
    note: str = ""  # 记录异常情况或额外说明。


@dataclass
class SingleTrialResult:  # 保存一次真实执行过的单次动作试验结果。
    kp: float  # 本次试验写入的位置环 P 参数。
    tracker: ResponseTracker  # 本次试验对应的 reference/feedback 响应曲线样本。
    metrics: TrackingMetrics  # 本次试验对应的评价指标。
    action_label: str = "single"  # 该结果对应的测试动作标签。
    repeat_index: int = 1  # 同一动作下的重复试验序号，从 1 开始。


@dataclass
class KpTrialResult:  # 保存某个 Kp 候选值的聚合摘要，并显式关联原始试验记录。
    kp: float  # 本次试验写入的位置环 P 参数。
    metrics: TrackingMetrics  # 该 Kp 聚合后的评价指标。
    display_trial: SingleTrialResult  # 用于绘图和人工复盘的代表性单次试验。
    raw_trials: list[SingleTrialResult]  # 该 Kp 下所有原始 trial，供 CSV 明细导出。
    repeat_count: int = 1  # 该 Kp 实际重复评估次数。
    search_cost_std: float = 0.0  # 多次重复时 search_cost 的样本标准差。
    action_label: str = "single"  # 该结果对应的测试动作标签，聚合结果记为 multi_action。


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
    joint_positions: Optional[dict[str, float]] = None  # setup 阶段可一次性命令同臂多个关节到固定姿态。


@dataclass
class AutoTuneResult:  # 自动调参的完整结果，替代多元组返回。
    best_kp: float  # 搜索到的最优 Kp。
    history: list  # 所有中间候选 Kp 的试验结果列表。
    original_kp: float  # 调参前的原始 Kp。
    baseline_result: "KpTrialResult"  # 原始 Kp 的基线试验结果。
    best_result: "KpTrialResult"  # 最终被选中的最优 Kp 聚合结果。
    action_labels: list[str]  # 本次自动调参使用的 3 个人工输入动作标签。
    reset_position: float  # 基线试验时使用的统一起始位置。


@dataclass
class GaussianProcessPrediction:  # 保存一维高斯过程在候选网格上的均值与标准差预测。
    xs: list[float]
    means: list[float]
    stds: list[float]


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


def tail_slice(values: list[float], ratio: float = STABLE_TAIL_RATIO) -> list[float]:  # 提取末尾稳态窗口样本，用于稳定性和平滑度分析。
    if not values:
        raise ValueError("cannot compute tail slice from an empty list")
    window = max(2, math.ceil(len(values) * ratio))
    return values[-window:]


def sequence_std(values: list[float]) -> float:  # 计算序列标准差，供尾部抖动判定使用。
    if len(values) < 2:
        return 0.0
    mean_value = sum(values) / len(values)
    variance = sum((value - mean_value) ** 2 for value in values) / len(values)
    return math.sqrt(max(variance, 0.0))


def adjacent_diff_rms(values: list[float]) -> float:  # 计算相邻样本差分 RMS，供尾部平滑度判定使用。
    if len(values) < 2:
        return 0.0
    diffs = [values[index + 1] - values[index] for index in range(len(values) - 1)]
    mean_square = sum(diff * diff for diff in diffs) / len(diffs)
    return math.sqrt(max(mean_square, 0.0))


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
            tail_std_rad=float("inf"),
            tail_peak_to_peak_rad=float("inf"),
            tail_diff_rms_rad=float("inf"),
            rmse_term=rmse / RMSE_TARGET_RAD,
            phase_lag_term=float("inf"),
            amplitude_ratio_term=float("inf"),
            smoothness_term=float("inf"),
            rmse_penalty=RMSE_EXCESS_PENALTY_WEIGHT * max(0.0, rmse / RMSE_TARGET_RAD - 1.0),
            phase_lag_penalty=float("inf"),
            amplitude_ratio_penalty=float("inf"),
            overshoot_penalty=0.0,
            search_cost=float("inf"),
            strict_ok=False,
            expand_safe=False,
            stable_ok=False,
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

    tail_positions = tail_slice(tracker.sample_positions)
    tail_std_rad = sequence_std(tail_positions)
    tail_peak_to_peak_rad = max(tail_positions) - min(tail_positions)
    tail_diff_rms_rad = adjacent_diff_rms(tail_positions)

    rmse_term = rmse / RMSE_TARGET_RAD  # 把 RMSE 归一化到“0.01 度目标值的多少倍”。
    phase_lag_term = (
        phase_lag_deg / PHASE_LAG_TARGET_DEG
        if math.isfinite(phase_lag_deg)
        else 1_000.0
    )  # 把相位滞后归一化到 2 度目标的倍数；不可定义时直接记为极大值。
    amplitude_ratio_term = amplitude_ratio_error / (1.0 - AMPLITUDE_RATIO_MIN)  # 把幅值比偏差归一化到 0.95 下限的容忍带宽。
    smoothness_term = (
        tail_std_rad / STABLE_TAIL_STD_MAX_RAD
        + tail_peak_to_peak_rad / STABLE_TAIL_PEAK_TO_PEAK_MAX_RAD
        + tail_diff_rms_rad / STABLE_TAIL_DIFF_RMS_MAX_RAD
    ) / 3.0

    base_cost = (
        RMSE_COST_WEIGHT * rmse_term
        + PHASE_LAG_COST_WEIGHT * phase_lag_term
        + AMPLITUDE_RATIO_COST_WEIGHT * amplitude_ratio_term
        + SMOOTHNESS_COST_WEIGHT * smoothness_term
    )
    rmse_penalty = RMSE_EXCESS_PENALTY_WEIGHT * max(0.0, rmse_term - 1.0)  # RMSE 超过 0.01 度目标后，按超出比例施加大罚分。
    phase_lag_penalty = PHASE_LAG_EXCESS_PENALTY_WEIGHT * max(0.0, phase_lag_term - 1.0)  # 相位滞后超过 2 度目标后，按超出比例施加罚分。
    amplitude_ratio_penalty = AMPLITUDE_RATIO_DEFICIT_PENALTY_WEIGHT * max(0.0, (AMPLITUDE_RATIO_MIN - amplitude_ratio) / AMPLITUDE_RATIO_MIN)  # 幅值比低于 0.95 后施加罚分。
    overshoot_penalty = OVERSHOOT_HARD_PENALTY if overshoot_rad >= OVERSHOOT_PENALTY_THRESHOLD else 0.0
    stable_ok = (
        tail_std_rad <= STABLE_TAIL_STD_MAX_RAD
        and tail_peak_to_peak_rad <= STABLE_TAIL_PEAK_TO_PEAK_MAX_RAD
        and tail_diff_rms_rad <= STABLE_TAIL_DIFF_RMS_MAX_RAD
    )
    stability_penalty = 0.0 if stable_ok else UNSTABLE_PENALTY
    search_cost = base_cost + rmse_penalty + phase_lag_penalty + amplitude_ratio_penalty + overshoot_penalty + stability_penalty
    strict_ok = (
        stable_ok
        and
        rmse <= STRICT_RMSE_MAX_RAD
        and phase_lag_deg <= STRICT_PHASE_LAG_MAX_DEG
        and amplitude_ratio >= STRICT_AMPLITUDE_RATIO_MIN
        and overshoot_rad <= OVERSHOOT_STRICT_MAX_RAD
    )  # 最终严格验收：现实可达的工程阈值。
    expand_safe = (
        stable_ok
        and
        overshoot_rad <= OVERSHOOT_EXPAND_MAX_RAD
        and amplitude_ratio >= AMPLITUDE_RATIO_MIN
    )  # 阶段过渡判定：允许小超调，但要求无明显过冲且幅值比达标。

    return TrackingMetrics(
        rmse=rmse,
        phase_lag_deg=phase_lag_deg,
        amplitude_ratio=amplitude_ratio,
        overshoot_rad=overshoot_rad,
        tail_std_rad=tail_std_rad,
        tail_peak_to_peak_rad=tail_peak_to_peak_rad,
        tail_diff_rms_rad=tail_diff_rms_rad,
        rmse_term=rmse_term,
        phase_lag_term=phase_lag_term,
        amplitude_ratio_term=amplitude_ratio_term,
        smoothness_term=smoothness_term,
        rmse_penalty=rmse_penalty,
        phase_lag_penalty=phase_lag_penalty,
        amplitude_ratio_penalty=amplitude_ratio_penalty,
        overshoot_penalty=overshoot_penalty,
        search_cost=search_cost,
        strict_ok=strict_ok,
        expand_safe=expand_safe,
        stable_ok=stable_ok,
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


def default_auto_tune_plot_path() -> str:  # 自动调参模式的默认图片基名，不再依赖命令行 position。
    return str(Path(__file__).with_name(f"{JOINT_NAME}_auto_tune.png"))


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
    baseline: SingleTrialResult,
    best: SingleTrialResult,
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
    baseline: SingleTrialResult,
    best: SingleTrialResult,
) -> tuple[tuple[float, float], tuple[float, float]]:  # 为基线和最优误差图计算统一坐标范围，便于横向比较误差量级。
    time_values = baseline.tracker.sample_times + best.tracker.sample_times
    error_values = baseline.tracker.sample_errors + best.tracker.sample_errors
    return _expand_range(min(time_values), max(time_values)), _expand_range(
        min(error_values),
        max(error_values),
    )


def default_kp_trials_raw_csv_path() -> str:  # 生成默认的 Kp 调参原始 trial CSV 路径。
    file_name = "kp_trials_raw.csv" if JOINT_NAME == DEFAULT_JOINT_NAME else f"{JOINT_NAME}_kp_trials_raw.csv"
    return str(Path(__file__).with_name(file_name))


def default_kp_trials_summary_csv_path() -> str:  # 生成默认的 Kp 调参聚合摘要 CSV 路径。
    file_name = "kp_trials_summary.csv" if JOINT_NAME == DEFAULT_JOINT_NAME else f"{JOINT_NAME}_kp_trials_summary.csv"
    return str(Path(__file__).with_name(file_name))


def comparison_action_plot_path(base_output: str, action_label: str) -> str:  # 生成分动作对比图路径。
    plot_base = Path(base_output)
    return str(plot_base.with_name(f"{plot_base.stem}_comparison_{action_label}.png"))


def preview_action_plot_path(base_output: str, action_label: str) -> str:  # 生成人工输入动作的单次预览图路径。
    plot_base = Path(base_output)
    return str(plot_base.with_name(f"{plot_base.stem}_{action_label}_preview.png"))


def find_raw_trial(
    trial: KpTrialResult,
    action_label: str,
    repeat_index: int,
) -> Optional[SingleTrialResult]:  # 从某个 Kp 聚合结果中选出指定动作和重复序号的原始 trial。
    for raw_trial in trial.raw_trials:
        if raw_trial.action_label == action_label and raw_trial.repeat_index == repeat_index:
            return raw_trial
    return None


def _trial_role(index: int, trial: KpTrialResult, best_kp: float) -> str:
    if index == 0:
        return "baseline"
    return "best_candidate" if abs(trial.kp - best_kp) <= POSITION_EPSILON else "candidate"


def save_kp_trials_csvs(
    baseline: KpTrialResult,
    history: list[KpTrialResult],
    best_kp: float,
    raw_output_path: str,
    summary_output_path: str,
) -> tuple[str, str]:  # 将原始 trial 明细与 Kp 聚合摘要分别落盘，避免混淆。
    rows = [baseline] + history
    raw_output = Path(raw_output_path)
    raw_output.parent.mkdir(parents=True, exist_ok=True)
    summary_output = Path(summary_output_path)
    summary_output.parent.mkdir(parents=True, exist_ok=True)

    raw_fieldnames = [
        "role",
        "kp",
        "action_label",
        "repeat_index",
        "rmse_rad",
        "phase_lag_deg",
        "amplitude_ratio",
        "overshoot_rad",
        "tail_std_rad",
        "tail_peak_to_peak_rad",
        "tail_diff_rms_rad",
        "rmse_term",
        "phase_lag_term",
        "amplitude_ratio_term",
        "smoothness_term",
        "rmse_penalty",
        "phase_lag_penalty",
        "amplitude_ratio_penalty",
        "overshoot_penalty",
        "search_cost",
        "strict_ok",
        "expand_safe",
        "stable_ok",
        "note",
        "initial_position_rad",
        "initial_reference_position_rad",
        "target_position_rad",
        "final_position_rad",
        "final_reference_position_rad",
        "sample_count",
        "summary_action_label",
        "summary_repeat_count",
        "summary_search_cost_std",
    ]
    with raw_output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=raw_fieldnames)
        writer.writeheader()
        for index, trial in enumerate(rows):
            role = _trial_role(index, trial, best_kp)
            for raw_trial in trial.raw_trials:
                writer.writerow(
                    {
                        "role": role,
                        "kp": f"{raw_trial.kp:.6f}",
                        "action_label": raw_trial.action_label,
                        "repeat_index": raw_trial.repeat_index,
                        "rmse_rad": f"{raw_trial.metrics.rmse:.6f}",
                        "phase_lag_deg": f"{raw_trial.metrics.phase_lag_deg:.6f}",
                        "amplitude_ratio": f"{raw_trial.metrics.amplitude_ratio:.6f}",
                        "overshoot_rad": f"{raw_trial.metrics.overshoot_rad:.6f}",
                        "tail_std_rad": f"{raw_trial.metrics.tail_std_rad:.6f}",
                        "tail_peak_to_peak_rad": f"{raw_trial.metrics.tail_peak_to_peak_rad:.6f}",
                        "tail_diff_rms_rad": f"{raw_trial.metrics.tail_diff_rms_rad:.6f}",
                        "rmse_term": f"{raw_trial.metrics.rmse_term:.6f}",
                        "phase_lag_term": f"{raw_trial.metrics.phase_lag_term:.6f}",
                        "amplitude_ratio_term": f"{raw_trial.metrics.amplitude_ratio_term:.6f}",
                        "smoothness_term": f"{raw_trial.metrics.smoothness_term:.6f}",
                        "rmse_penalty": f"{raw_trial.metrics.rmse_penalty:.6f}",
                        "phase_lag_penalty": f"{raw_trial.metrics.phase_lag_penalty:.6f}",
                        "amplitude_ratio_penalty": f"{raw_trial.metrics.amplitude_ratio_penalty:.6f}",
                        "overshoot_penalty": f"{raw_trial.metrics.overshoot_penalty:.6f}",
                        "search_cost": f"{raw_trial.metrics.search_cost:.6f}",
                        "strict_ok": str(raw_trial.metrics.strict_ok),
                        "expand_safe": str(raw_trial.metrics.expand_safe),
                        "stable_ok": str(raw_trial.metrics.stable_ok),
                        "note": raw_trial.metrics.note,
                        "initial_position_rad": f"{raw_trial.tracker.initial_position:.6f}",
                        "initial_reference_position_rad": f"{raw_trial.tracker.initial_reference_position:.6f}",
                        "target_position_rad": f"{raw_trial.tracker.target_position:.6f}",
                        "final_position_rad": (
                            f"{raw_trial.tracker.last_position:.6f}" if raw_trial.tracker.last_position is not None else ""
                        ),
                        "final_reference_position_rad": (
                            f"{raw_trial.tracker.last_reference_position:.6f}"
                            if raw_trial.tracker.last_reference_position is not None
                            else ""
                        ),
                        "sample_count": len(raw_trial.tracker.sample_times),
                        "summary_action_label": trial.action_label,
                        "summary_repeat_count": trial.repeat_count,
                        "summary_search_cost_std": f"{trial.search_cost_std:.6f}",
                    }
                )

    summary_fieldnames = [
        "role",
        "kp",
        "action_label",
        "rmse_rad",
        "phase_lag_deg",
        "amplitude_ratio",
        "overshoot_rad",
        "tail_std_rad",
        "tail_peak_to_peak_rad",
        "tail_diff_rms_rad",
        "rmse_term",
        "phase_lag_term",
        "amplitude_ratio_term",
        "smoothness_term",
        "rmse_penalty",
        "phase_lag_penalty",
        "amplitude_ratio_penalty",
        "overshoot_penalty",
        "search_cost",
        "search_cost_std",
        "repeat_count",
        "strict_ok",
        "expand_safe",
        "stable_ok",
        "note",
        "raw_trial_count",
        "display_action_label",
        "display_repeat_index",
        "display_initial_position_rad",
        "display_initial_reference_position_rad",
        "display_target_position_rad",
        "display_final_position_rad",
        "display_final_reference_position_rad",
        "display_sample_count",
    ]
    with summary_output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=summary_fieldnames)
        writer.writeheader()
        for index, trial in enumerate(rows):
            role = _trial_role(index, trial, best_kp)
            display_trial = trial.display_trial
            writer.writerow(
                {
                    "role": role,
                    "kp": f"{trial.kp:.6f}",
                    "action_label": trial.action_label,
                    "rmse_rad": f"{trial.metrics.rmse:.6f}",
                    "phase_lag_deg": f"{trial.metrics.phase_lag_deg:.6f}",
                    "amplitude_ratio": f"{trial.metrics.amplitude_ratio:.6f}",
                    "overshoot_rad": f"{trial.metrics.overshoot_rad:.6f}",
                    "tail_std_rad": f"{trial.metrics.tail_std_rad:.6f}",
                    "tail_peak_to_peak_rad": f"{trial.metrics.tail_peak_to_peak_rad:.6f}",
                    "tail_diff_rms_rad": f"{trial.metrics.tail_diff_rms_rad:.6f}",
                    "rmse_term": f"{trial.metrics.rmse_term:.6f}",
                    "phase_lag_term": f"{trial.metrics.phase_lag_term:.6f}",
                    "amplitude_ratio_term": f"{trial.metrics.amplitude_ratio_term:.6f}",
                    "smoothness_term": f"{trial.metrics.smoothness_term:.6f}",
                    "rmse_penalty": f"{trial.metrics.rmse_penalty:.6f}",
                    "phase_lag_penalty": f"{trial.metrics.phase_lag_penalty:.6f}",
                    "amplitude_ratio_penalty": f"{trial.metrics.amplitude_ratio_penalty:.6f}",
                    "overshoot_penalty": f"{trial.metrics.overshoot_penalty:.6f}",
                    "search_cost": f"{trial.metrics.search_cost:.6f}",
                    "search_cost_std": f"{trial.search_cost_std:.6f}",
                    "repeat_count": trial.repeat_count,
                    "strict_ok": str(trial.metrics.strict_ok),
                    "expand_safe": str(trial.metrics.expand_safe),
                    "stable_ok": str(trial.metrics.stable_ok),
                    "note": trial.metrics.note,
                    "raw_trial_count": len(trial.raw_trials),
                    "display_action_label": display_trial.action_label,
                    "display_repeat_index": display_trial.repeat_index,
                    "display_initial_position_rad": f"{display_trial.tracker.initial_position:.6f}",
                    "display_initial_reference_position_rad": f"{display_trial.tracker.initial_reference_position:.6f}",
                    "display_target_position_rad": f"{display_trial.tracker.target_position:.6f}",
                    "display_final_position_rad": (
                        f"{display_trial.tracker.last_position:.6f}"
                        if display_trial.tracker.last_position is not None
                        else ""
                    ),
                    "display_final_reference_position_rad": (
                        f"{display_trial.tracker.last_reference_position:.6f}"
                        if display_trial.tracker.last_reference_position is not None
                        else ""
                    ),
                    "display_sample_count": len(display_trial.tracker.sample_times),
                }
            )
    return str(raw_output), str(summary_output)


def _normal_pdf(value: float) -> float:  # 标准正态分布概率密度函数，供 EI 计算使用。
    return math.exp(-0.5 * value * value) / math.sqrt(2.0 * math.pi)


def _normal_cdf(value: float) -> float:  # 标准正态分布累计分布函数，供 EI 计算使用。
    return 0.5 * (1.0 + math.erf(value / math.sqrt(2.0)))


def _estimate_gp_length_scale(xs: list[float], low: float, high: float) -> float:  # 用区间宽度和样本密度估计一维 GP 的长度尺度。
    span = max(high - low, KP_INTERVAL_TOLERANCE)
    if len(xs) < 2:
        return max(span / 3.0, KP_INTERVAL_TOLERANCE)
    sorted_xs = sorted(xs)
    gaps = [sorted_xs[index + 1] - sorted_xs[index] for index in range(len(sorted_xs) - 1)]
    mean_gap = sum(gaps) / len(gaps)
    return max(span / 6.0, mean_gap * 1.5, KP_INTERVAL_TOLERANCE)


def _rbf_kernel(x1: float, x2: float, length_scale: float) -> float:  # 一维 RBF 核，用于拟合 Kp->cost 的平滑代理模型。
    delta = (x1 - x2) / max(length_scale, KP_INTERVAL_TOLERANCE)
    return math.exp(-0.5 * delta * delta)


def _solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:  # 用高斯消元求解线性方程组，避免额外依赖 numpy。
    size = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for pivot_index in range(size):
        pivot_row = max(range(pivot_index, size), key=lambda row_index: abs(augmented[row_index][pivot_index]))
        if abs(augmented[pivot_row][pivot_index]) < KP_BAYES_JITTER:
            raise RuntimeError("Gaussian process linear system is singular")
        if pivot_row != pivot_index:
            augmented[pivot_index], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_index]

        pivot = augmented[pivot_index][pivot_index]
        for column_index in range(pivot_index, size + 1):
            augmented[pivot_index][column_index] /= pivot

        for row_index in range(size):
            if row_index == pivot_index:
                continue
            factor = augmented[row_index][pivot_index]
            if abs(factor) < KP_BAYES_JITTER:
                continue
            for column_index in range(pivot_index, size + 1):
                augmented[row_index][column_index] -= factor * augmented[pivot_index][column_index]

    return [augmented[row_index][size] for row_index in range(size)]


def normalize_costs(costs: list[float]) -> tuple[list[float], float, float]:  # 对 cost 做标准化，提升 GP/EI 的数值稳定性。
    if not costs:
        raise ValueError("Cannot normalize an empty cost list")
    mean_cost = sum(costs) / len(costs)
    variance = sum((cost - mean_cost) ** 2 for cost in costs) / len(costs)
    std_cost = math.sqrt(max(variance, KP_BAYES_JITTER))
    normalized = [(cost - mean_cost) / std_cost for cost in costs]
    return normalized, mean_cost, std_cost


def fit_gaussian_process(
    xs: list[float],
    ys: list[float],
    grid: list[float],
    low: float,
    high: float,
) -> GaussianProcessPrediction:  # 在一维网格上拟合高斯过程并输出均值/标准差，供 EI 选点使用。
    if len(xs) != len(ys) or not xs:
        raise ValueError("Gaussian process requires non-empty paired observations")

    length_scale = _estimate_gp_length_scale(xs, low, high)
    covariance = []
    for row_index, x_row in enumerate(xs):
        row = []
        for column_index, x_col in enumerate(xs):
            value = _rbf_kernel(x_row, x_col, length_scale)
            if row_index == column_index:
                value += KP_BAYES_NOISE_VARIANCE + KP_BAYES_JITTER
            row.append(value)
        covariance.append(row)

    alpha = _solve_linear_system(covariance, ys)
    means: list[float] = []
    stds: list[float] = []
    for x_star in grid:
        k_star = [_rbf_kernel(x_star, x_train, length_scale) for x_train in xs]
        mean = sum(weight * target for weight, target in zip(k_star, alpha))
        v = _solve_linear_system(covariance, k_star)
        variance = max(
            KP_BAYES_JITTER,
            1.0 + KP_BAYES_NOISE_VARIANCE - sum(left * right for left, right in zip(k_star, v)),
        )
        means.append(mean)
        stds.append(math.sqrt(variance))

    return GaussianProcessPrediction(xs=grid, means=means, stds=stds)


def expected_improvement(
    prediction: GaussianProcessPrediction,
    best_cost: float,
) -> list[float]:  # 基于 GP 预测计算每个候选点的 Expected Improvement。
    improvements: list[float] = []
    for mean, std in zip(prediction.means, prediction.stds):
        if std <= KP_BAYES_JITTER:
            improvements.append(max(0.0, best_cost - mean))
            continue
        z_value = (best_cost - mean) / std
        ei = (best_cost - mean) * _normal_cdf(z_value) + std * _normal_pdf(z_value)
        improvements.append(max(0.0, ei))
    return improvements


def choose_next_kp_via_bayes(
    observed_kps: list[float],
    observed_costs: list[float],
    low: float,
    high: float,
) -> tuple[float, float]:  # 用一维 GP+EI 从当前区间里选出下一个最值得评估的 Kp。
    if not observed_kps:
        raise ValueError("At least one observation is required for Bayesian selection")

    if high - low < KP_INTERVAL_TOLERANCE:
        midpoint = low + (high - low) / 2.0
        return midpoint, 0.0

    grid = [
        low + (high - low) * index / (KP_BAYES_GRID_SIZE - 1)
        for index in range(KP_BAYES_GRID_SIZE)
    ]
    normalized_costs, _, _ = normalize_costs(observed_costs)
    prediction = fit_gaussian_process(observed_kps, normalized_costs, grid, low, high)
    eis = expected_improvement(prediction, min(normalized_costs))

    ranked_indices = sorted(range(len(grid)), key=lambda index: eis[index], reverse=True)
    for index in ranked_indices:
        candidate = grid[index]
        if min(abs(candidate - observed) for observed in observed_kps) >= KP_DUPLICATE_TOLERANCE:
            return candidate, eis[index]

    midpoint = low + (high - low) / 2.0
    return midpoint, 0.0


def aggregate_kp_trials(kp: float, trials: list[SingleTrialResult]) -> KpTrialResult:  # 将同一 Kp 的多次重复试验聚合成一个代表结果，便于后续统一比较。
    if not trials:
        raise ValueError("Cannot aggregate an empty trial list")
    if len(trials) == 1:
        trial = trials[0]
        return KpTrialResult(
            kp=kp,
            metrics=trial.metrics,
            display_trial=trial,
            raw_trials=[trial],
            repeat_count=1,
            search_cost_std=0.0,
            action_label=trial.action_label,
        )

    best_single = min(trials, key=lambda trial: trial.metrics.search_cost)
    mean_search_cost = sum(trial.metrics.search_cost for trial in trials) / len(trials)
    variance = sum((trial.metrics.search_cost - mean_search_cost) ** 2 for trial in trials) / len(trials)

    aggregated_metrics = TrackingMetrics(
        rmse=sum(trial.metrics.rmse for trial in trials) / len(trials),
        phase_lag_deg=sum(trial.metrics.phase_lag_deg for trial in trials) / len(trials),
        amplitude_ratio=sum(trial.metrics.amplitude_ratio for trial in trials) / len(trials),
        overshoot_rad=sum(trial.metrics.overshoot_rad for trial in trials) / len(trials),
        tail_std_rad=sum(trial.metrics.tail_std_rad for trial in trials) / len(trials),
        tail_peak_to_peak_rad=sum(trial.metrics.tail_peak_to_peak_rad for trial in trials) / len(trials),
        tail_diff_rms_rad=sum(trial.metrics.tail_diff_rms_rad for trial in trials) / len(trials),
        rmse_term=sum(trial.metrics.rmse_term for trial in trials) / len(trials),
        phase_lag_term=sum(trial.metrics.phase_lag_term for trial in trials) / len(trials),
        amplitude_ratio_term=sum(trial.metrics.amplitude_ratio_term for trial in trials) / len(trials),
        smoothness_term=sum(trial.metrics.smoothness_term for trial in trials) / len(trials),
        rmse_penalty=sum(trial.metrics.rmse_penalty for trial in trials) / len(trials),
        phase_lag_penalty=sum(trial.metrics.phase_lag_penalty for trial in trials) / len(trials),
        amplitude_ratio_penalty=sum(trial.metrics.amplitude_ratio_penalty for trial in trials) / len(trials),
        overshoot_penalty=max(trial.metrics.overshoot_penalty for trial in trials),
        search_cost=mean_search_cost,
        strict_ok=all(trial.metrics.strict_ok for trial in trials),
        expand_safe=all(trial.metrics.expand_safe for trial in trials),
        stable_ok=all(trial.metrics.stable_ok for trial in trials),
        note="aggregated_from_repeats",
    )
    return KpTrialResult(
        kp=kp,
        metrics=aggregated_metrics,
        display_trial=best_single,
        raw_trials=list(trials),
        repeat_count=len(trials),
        search_cost_std=math.sqrt(max(variance, 0.0)),
        action_label=best_single.action_label,
    )


def aggregate_action_trials(kp: float, trials: list[KpTrialResult]) -> KpTrialResult:  # 将多个测试动作的聚合结果再次聚合成一个 Kp 级别的联合评价结果。
    if not trials:
        raise ValueError("Cannot aggregate an empty action trial list")
    best_single = min(
        trials,
        key=lambda trial: trial.metrics.search_cost + SEARCH_COST_STD_WEIGHT * trial.search_cost_std,
    )
    display_trial = best_single.display_trial  # 当前动作均为人工输入标签，直接展示综合评分最优的代表试验。
    mean_search_cost = sum(trial.metrics.search_cost for trial in trials) / len(trials)
    variance = sum((trial.metrics.search_cost - mean_search_cost) ** 2 for trial in trials) / len(trials)

    aggregated_metrics = TrackingMetrics(
        rmse=sum(trial.metrics.rmse for trial in trials) / len(trials),
        phase_lag_deg=sum(trial.metrics.phase_lag_deg for trial in trials) / len(trials),
        amplitude_ratio=sum(trial.metrics.amplitude_ratio for trial in trials) / len(trials),
        overshoot_rad=sum(trial.metrics.overshoot_rad for trial in trials) / len(trials),
        tail_std_rad=sum(trial.metrics.tail_std_rad for trial in trials) / len(trials),
        tail_peak_to_peak_rad=sum(trial.metrics.tail_peak_to_peak_rad for trial in trials) / len(trials),
        tail_diff_rms_rad=sum(trial.metrics.tail_diff_rms_rad for trial in trials) / len(trials),
        rmse_term=sum(trial.metrics.rmse_term for trial in trials) / len(trials),
        phase_lag_term=sum(trial.metrics.phase_lag_term for trial in trials) / len(trials),
        amplitude_ratio_term=sum(trial.metrics.amplitude_ratio_term for trial in trials) / len(trials),
        smoothness_term=sum(trial.metrics.smoothness_term for trial in trials) / len(trials),
        rmse_penalty=sum(trial.metrics.rmse_penalty for trial in trials) / len(trials),
        phase_lag_penalty=sum(trial.metrics.phase_lag_penalty for trial in trials) / len(trials),
        amplitude_ratio_penalty=sum(trial.metrics.amplitude_ratio_penalty for trial in trials) / len(trials),
        overshoot_penalty=max(trial.metrics.overshoot_penalty for trial in trials),
        search_cost=mean_search_cost,
        strict_ok=all(trial.metrics.strict_ok for trial in trials),
        expand_safe=all(trial.metrics.expand_safe for trial in trials),
        stable_ok=all(trial.metrics.stable_ok for trial in trials),
        note="aggregated_from_multi_actions",
    )
    return KpTrialResult(
        kp=kp,
        metrics=aggregated_metrics,
        display_trial=display_trial,
        raw_trials=[raw_trial for trial in trials for raw_trial in trial.raw_trials],
        repeat_count=sum(trial.repeat_count for trial in trials),
        search_cost_std=math.sqrt(max(variance, 0.0)),
        action_label="multi_action",
    )


def is_safe_kp_trial(trial: KpTrialResult) -> bool:  # 判断某个 Kp 试验是否满足“完全达标”的严格安全条件。
    return trial.metrics.strict_ok


def is_expand_safe(trial: KpTrialResult) -> bool:  # 判断某个 Kp 试验是否满足“可进入下一阶段或作为回退候选”的宽松安全条件。
    return trial.metrics.expand_safe


def allocate_stage_iteration_budgets(total_iterations: int) -> list[int]:  # 将总预算分配到 3 个自动调参阶段，保证前期粗搜、后期精调都有足够次数。
    total_iterations = max(total_iterations, AUTO_TUNE_MIN_ITERATIONS)
    budgets = [KP_BAYES_INITIAL_SAMPLE_COUNT] * AUTO_TUNE_MAX_STAGES
    remaining = total_iterations - AUTO_TUNE_MIN_ITERATIONS
    if remaining == 0:
        return budgets

    weights = list(AUTO_TUNE_STAGE_ITERATIONS)
    weight_total = sum(weights)
    for index in range(AUTO_TUNE_MAX_STAGES):
        share = (remaining * weights[index]) // weight_total
        budgets[index] += share
    distributed = sum(budgets) - AUTO_TUNE_MIN_ITERATIONS
    leftover = remaining - distributed
    index = AUTO_TUNE_MAX_STAGES - 1
    while leftover > 0:
        budgets[index] += 1
        leftover -= 1
        index = (index - 1) % AUTO_TUNE_MAX_STAGES
    return budgets


def next_stage_bounds(center_kp: float, hard_max: float, stage_index: int) -> tuple[float, float]:  # 围绕上一阶段最优点生成下一阶段自动收窄区间。
    half_width = AUTO_TUNE_STAGE_HALF_WIDTHS[min(stage_index, len(AUTO_TUNE_STAGE_HALF_WIDTHS) - 1)]
    low = max(0.0, center_kp - half_width)
    high = min(hard_max, center_kp + half_width)
    if low >= high:
        high = min(hard_max, center_kp + KP_INTERVAL_TOLERANCE)
        low = max(0.0, high - 2.0 * KP_INTERVAL_TOLERANCE)
    return low, high


def centered_initial_kp_bounds(center_kp: float, half_width: float, hard_max: float) -> tuple[float, float]:  # 初始阶段默认围绕当前 Kp 居中取区间，若触边则尽量平移保持宽度。
    effective_half_width = max(half_width, KP_INTERVAL_TOLERANCE)
    low = center_kp - effective_half_width
    high = center_kp + effective_half_width
    if low < 0.0:
        high = min(hard_max, high - low)
        low = 0.0
    if high > hard_max:
        low = max(0.0, low - (high - hard_max))
        high = hard_max
    if low >= high:
        high = min(hard_max, center_kp + KP_INTERVAL_TOLERANCE)
        low = max(0.0, high - 2.0 * KP_INTERVAL_TOLERANCE)
    return low, high


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
    baseline: SingleTrialResult,
    best: SingleTrialResult,
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
            f"{title}\nKp={trial.kp:.4f}, RMSE={trial.metrics.rmse:.6f}, Cost={trial.metrics.search_cost:.6f}"
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
    joint_positions = req.joint_positions or {JOINT_NAME: req.position}
    joint_targets = ",".join(
        f"{joint_name}:{position:.12g}"
        for joint_name, position in ordered_joint_positions(joint_positions)
    )
    parts = [
        f"group={COMMAND_GROUP}",
        "type=joints",
        f"joints={joint_targets}",
        f"vel={req.vel}",
        f"acc={req.acc}",
        f"pipeline={req.pipeline}",
        f"planner_id={req.planner_id}",
        f"exec={1 if req.exec_motion else 0}",
    ]
    return " ".join(parts)


def parse_args() -> argparse.Namespace:  # 定义并解析命令行参数。
    parser = argparse.ArgumentParser(  # 创建参数解析器，并设置帮助说明。
        description="Control one arm joint via /cli_controller/execute_command and optionally auto-tune its Kp."
    )
    parser.add_argument(  # 添加位置参数 `position`。
        "position",
        nargs="?",
        type=float,
        help="Target joint position in radians. Required for single-motion mode; ignored by auto-tune mode.",
    )
    parser.add_argument(  # 添加可选参数 `--joint-name`，用于选择要控制和调参的关节。
        "--joint-name",
        default=DEFAULT_JOINT_NAME,
        help=f"Arm joint to control. Default: {DEFAULT_JOINT_NAME}",
    )
    parser.add_argument(  # 添加可选参数 `--group`，用于覆盖 MoveIt group。
        "--group",
        default=None,
        help="MoveIt group used by ExecuteCommand. Default: inferred from --joint-name",
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
        default=None,
        help="Controller state topic used to plot reference and feedback. Default: inferred from --joint-name",
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
    parser.add_argument(  # 添加可选参数 `--auto-tune-actions`，用于非交互指定 3 个测试角度。
        "--auto-tune-actions",
        default=None,
        help="Comma-separated target positions for Kp auto-tune, e.g. 0.1,0.3,0.5. Default: prompt manually",
    )
    parser.add_argument(  # 添加可选参数 `--setup-joints`，用于每个测试动作前先移动同臂 6 个辅助关节到固定姿态。
        "--setup-joints",
        default=None,
        help=(
            "Comma-separated same-arm helper joint positions applied before each auto-tune trial, "
            "e.g. left_arm_1_joint:0.1,left_arm_3_joint:-0.2. "
            "Must define exactly the other 6 joints in the same arm as --joint-name."
        ),
    )
    parser.add_argument(  # 添加可选参数 `--kp-alias`，用于指定 EtherCAT alias。
        "--kp-alias",
        type=int,
        default=None,
        help="EtherCAT alias of the joint drive. Default: inferred from --joint-name",
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
    parser.add_argument(  # 添加可选参数 `--kp-range`，用于显式指定绝对浮动量；默认改用当前 Kp 的相对小区间。
        "--kp-range",
        type=float,
        default=None,
        help=f"Absolute Kp search offset from current Kp. Default: center the initial stage around current Kp with +/-{DEFAULT_KP_RELATIVE_RANGE_RATIO * 100:.0f}%% if kp-min/max are not set",
    )
    parser.add_argument(  # 添加可选参数 `--kp-hard-max`，限制自动调参时允许探索的绝对最高 Kp。
        "--kp-hard-max",
        type=float,
        default=0.0,
        help=f"Absolute hard upper limit for Kp auto-tuning. 0 means auto-derive from original_kp * {DEFAULT_KP_HARD_MAX_FACTOR}, then clamp to <= {DEFAULT_KP_HARD_MAX_LIMIT}",
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
        default=12,
        help="Maximum number of Kp candidate evaluations across the 3-stage auto-tuning flow. Minimum: 9. Default: 12",
    )
    parser.add_argument(  # 添加可选参数 `--kp-raw-csv-output`，用于指定原始 trial CSV 路径。
        "--kp-raw-csv-output",
        default=None,
        help="CSV output path for raw Kp trials. Default: auto-generated in the script directory",
    )
    parser.add_argument(  # 添加可选参数 `--kp-summary-csv-output`，用于指定聚合摘要 CSV 路径。
        "--kp-summary-csv-output",
        default=None,
        help="CSV output path for aggregated Kp trial summaries. Default: auto-generated in the script directory",
    )
    return parser.parse_args()


def main() -> int:  # 主函数，返回进程退出码。
    args = parse_args()
    try:
        configure_joint_context(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    try:
        setup_joint_positions = (
            parse_joint_position_map(args.setup_joints) if args.setup_joints else {}
        )
        validate_setup_joints(JOINT_NAME, setup_joint_positions)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    args.setup_joint_positions = setup_joint_positions

    if not args.auto_tune_kp and args.position is None:
        print("position is required unless --auto-tune-kp is used", file=sys.stderr)
        return 2
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
            super().__init__(f"move_{sanitize_identifier(JOINT_NAME)}_client")
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

        def clear_response_tracking(self) -> None:  # 一次 trial 结束后立即停止向旧 tracker 追加样本。
            self._response_tracker = None

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
        print(f"{header}tail_std: {metrics.tail_std_rad:.6f} rad")
        print(f"{header}tail_peak_to_peak: {metrics.tail_peak_to_peak_rad:.6f} rad")
        print(f"{header}tail_diff_rms: {metrics.tail_diff_rms_rad:.6f} rad")
        print(f"{header}rmse_term: {metrics.rmse_term:.6f}")
        print(f"{header}phase_lag_term: {metrics.phase_lag_term:.6f}")
        print(f"{header}amplitude_ratio_term: {metrics.amplitude_ratio_term:.6f}")
        print(f"{header}smoothness_term: {metrics.smoothness_term:.6f}")
        print(f"{header}rmse_penalty: {metrics.rmse_penalty:.6f}")
        print(f"{header}phase_lag_penalty: {metrics.phase_lag_penalty:.6f}")
        print(f"{header}amplitude_ratio_penalty: {metrics.amplitude_ratio_penalty:.6f}")
        print(f"{header}overshoot_penalty: {metrics.overshoot_penalty:.6f}")
        print(f"{header}search_cost: {metrics.search_cost:.6f}")
        print(f"{header}strict_ok: {metrics.strict_ok}")
        print(f"{header}expand_safe: {metrics.expand_safe}")
        print(f"{header}stable_ok: {metrics.stable_ok}")
        if metrics.note:
            print(f"{header}note: {metrics.note}")

    def print_trial_summary(trial: KpTrialResult, prefix: str = "") -> None:  # 补充打印重复试验的聚合信息，便于现场判断该 Kp 是否稳定。
        header = f"{prefix}" if prefix else ""
        print(f"{header}repeat_count: {trial.repeat_count}")
        print(f"{header}search_cost_std: {trial.search_cost_std:.6f}")

    def run_single_motion_trial(req: MotionRequest) -> tuple[str, object, float, ResponseTracker, TrackingMetrics]:  # 执行一次运动并返回完整测量结果。
        try:
            command, response, initial_position, tracker = node.call(req)
            metrics = finalize_trial(tracker, response, req.observe_window)
            return command, response, initial_position, tracker, metrics
        finally:
            node.clear_response_tracking()

    def setup_joint_positions_for(target_position: float) -> Optional[dict[str, float]]:  # 生成“辅助关节固定 + 被调关节回位”的 setup 命令目标。
        if not args.setup_joint_positions:
            return None
        joint_positions = dict(args.setup_joint_positions)
        joint_positions[JOINT_NAME] = target_position
        return joint_positions

    def move_joint_to(position: float, base_req: MotionRequest) -> None:  # 每个 trial 前回到统一测试姿态；setup 阶段不参与评分。
        reset_req = replace(
            base_req,
            position=position,
            joint_positions=setup_joint_positions_for(position),
        )
        _, _, _, _, _ = run_single_motion_trial(reset_req)

    def build_seeded_action_requests(
        action_positions: list[float],
        base_req: MotionRequest,
        reset_position: float,
        kp: float,
        label_prefix: str = "input",
    ) -> tuple[list[tuple[str, float, MotionRequest]], dict[str, list[SingleTrialResult]]]:  # 将目标角度转成测试动作，并先用原始 Kp 预跑一次。
        if not action_positions:
            raise RuntimeError("No auto-tune action positions were provided")
        action_specs: list[tuple[str, float, MotionRequest]] = []
        seeded_trials: dict[str, list[SingleTrialResult]] = {}
        preview_base_output = args.plot_output or default_auto_tune_plot_path()
        for index, action_position in enumerate(action_positions):
            action_label = f"{label_prefix}_{index + 1}"
            print(f"{action_label} target position (rad): {action_position:.6f}")
            action_req = replace(base_req, position=action_position)
            action_specs.append((action_label, reset_position, action_req))

            move_joint_to(reset_position, base_req)
            write_kp(args.kp_alias, args.kp_index, args.kp_subindex, kp)
            _, _, _, tracker, metrics = run_single_motion_trial(action_req)
            preview_trial = SingleTrialResult(
                kp=kp,
                tracker=tracker,
                metrics=metrics,
                action_label=action_label,
                repeat_index=1,
            )
            seeded_trials[action_label] = [preview_trial]
            print_metrics(metrics, prefix=f"{action_label} preview ")

            if not args.no_plot and tracker.command_time is not None:
                preview_output = preview_action_plot_path(preview_base_output, action_label)
                try:
                    preview_path = save_response_plot(tracker, preview_output, metrics.rmse)
                    print(f"{action_label} preview_plot: {preview_path}")
                except Exception as exc:
                    print(f"{action_label} preview_plot: unavailable ({exc})")

        return action_specs, seeded_trials

    def prompt_manual_action_requests(
        base_req: MotionRequest,
        reset_position: float,
        kp: float,
    ) -> tuple[list[tuple[str, float, MotionRequest]], dict[str, list[SingleTrialResult]]]:  # 依次提示用户输入 3 个动作角度，并立即运行一次生成预览图。
        action_positions: list[float] = []
        for index in range(DEFAULT_AUTO_TUNE_ACTION_COUNT):
            action_label = f"input_{index + 1}"
            while True:
                raw_value = input(f"{action_label} target position (rad): ").strip()
                try:
                    action_positions.append(float(raw_value))
                except ValueError:
                    print(f"{action_label}: invalid float value {raw_value!r}, please re-enter")
                    continue
                break
        return build_seeded_action_requests(action_positions, base_req, reset_position, kp)

    def evaluate_kp_candidate(
        kp: float,
        action_specs: list[tuple[str, float, MotionRequest]],
        seeded_trials_by_action: Optional[dict[str, list[SingleTrialResult]]] = None,
    ) -> KpTrialResult:  # 对某个候选 Kp 执行一组人工输入动作的闭环试验，并计算总代价。
        action_trials: list[KpTrialResult] = []
        for action_label, action_start, action_req in action_specs:
            repeated_trials = list((seeded_trials_by_action or {}).get(action_label, []))
            for repeat_index in range(len(repeated_trials), DEFAULT_KP_REPEATS):
                move_joint_to(action_start, action_req)
                write_kp(args.kp_alias, args.kp_index, args.kp_subindex, kp)
                _, _, _, tracker, metrics = run_single_motion_trial(action_req)
                repeated_trials.append(
                    SingleTrialResult(
                        kp=kp,
                        tracker=tracker,
                        metrics=metrics,
                        action_label=action_label,
                        repeat_index=repeat_index + 1,
                    )
                )
                if DEFAULT_KP_REPEATS > 1:
                    print(
                        f"kp_repeat[{repeat_index + 1}/{DEFAULT_KP_REPEATS}]: kp={kp:.6f} action={action_label} search_cost={metrics.search_cost:.6f}"
                    )
            action_trial = aggregate_kp_trials(kp, repeated_trials)
            action_trial.action_label = action_label
            action_trials.append(action_trial)
        return aggregate_action_trials(kp, action_trials)

    def auto_tune_kp(req: MotionRequest) -> AutoTuneResult:  # 在给定区间内自动搜索一个代价更低的位置环 Kp。
        nonlocal kp_restore_value
        if args.kp_iterations < AUTO_TUNE_MIN_ITERATIONS:
            raise RuntimeError(
                f"--kp-iterations must be at least {AUTO_TUNE_MIN_ITERATIONS} to complete the 3-stage auto-tuning flow"
            )

        original_kp = read_kp(args.kp_alias, args.kp_index, args.kp_subindex)
        kp_restore_value = original_kp
        hard_max = args.kp_hard_max
        if hard_max <= 0.0:
            hard_max = min(original_kp * DEFAULT_KP_HARD_MAX_FACTOR, DEFAULT_KP_HARD_MAX_LIMIT)
            print(
                f"kp_hard_max not specified, auto-derived: {hard_max:.6f} "
                f"(original_kp * {DEFAULT_KP_HARD_MAX_FACTOR}, clamped to <= {DEFAULT_KP_HARD_MAX_LIMIT})"
            )
        hard_max = min(hard_max, DEFAULT_KP_HARD_MAX_LIMIT)
        if original_kp > hard_max:
            raise RuntimeError(
                f"Current Kp {original_kp:.6f} exceeds the configured hard max {hard_max:.6f}"
            )

        if args.kp_min is not None:
            low = max(0.0, args.kp_min)
        elif args.kp_range is not None:
            low = max(0.0, original_kp - args.kp_range)
        else:
            default_half_width = max(abs(original_kp) * DEFAULT_KP_RELATIVE_RANGE_RATIO, KP_INTERVAL_TOLERANCE)
            low, _ = centered_initial_kp_bounds(original_kp, default_half_width, hard_max)

        if args.kp_max is not None:
            high = min(args.kp_max, hard_max)
        elif args.kp_range is not None:
            high = min(original_kp + args.kp_range, hard_max)
        else:
            default_half_width = max(abs(original_kp) * DEFAULT_KP_RELATIVE_RANGE_RATIO, KP_INTERVAL_TOLERANCE)
            _, high = centered_initial_kp_bounds(original_kp, default_half_width, hard_max)

        if low >= high:
            raise RuntimeError(f"Kp search interval is empty or inverted after hard-max clamp: [{low:.6f}, {high:.6f}]")
        print(
            f"kp_original: {original_kp:.6f}, search_interval: [{low:.6f}, {high:.6f}], hard_max: {hard_max:.6f}"
            )
        _, reset_position = node.wait_for_controller_state(req.state_timeout)
        if args.auto_tune_actions is not None:
            action_positions = parse_action_positions(args.auto_tune_actions)
            action_specs, baseline_seeded_trials = build_seeded_action_requests(
                action_positions,
                req,
                reset_position,
                original_kp,
            )
        else:
            action_specs, baseline_seeded_trials = prompt_manual_action_requests(req, reset_position, original_kp)
        baseline_result = evaluate_kp_candidate(original_kp, action_specs, baseline_seeded_trials)
        print_metrics(baseline_result.metrics, prefix=f"kp={original_kp:.6f} ")
        print_trial_summary(baseline_result, prefix=f"kp={original_kp:.6f} ")

        history: list[KpTrialResult] = []
        best_search: Optional[KpTrialResult] = baseline_result
        best_strict: Optional[KpTrialResult] = baseline_result if is_safe_kp_trial(baseline_result) else None
        best_expandable: Optional[KpTrialResult] = baseline_result if is_expand_safe(baseline_result) else None
        evaluated_trials: dict[int, KpTrialResult] = {}
        evaluated_trials[round(original_kp, 6)] = baseline_result

        def register_trial(trial: KpTrialResult) -> None:
            nonlocal best_search, best_strict, best_expandable
            trial_key = round(trial.kp, 6)
            history.append(trial)
            evaluated_trials[trial_key] = trial
            if best_search is None or trial.metrics.search_cost < best_search.metrics.search_cost:
                best_search = trial
            if is_safe_kp_trial(trial) and (best_strict is None or trial.metrics.search_cost < best_strict.metrics.search_cost):
                best_strict = trial
            if is_expand_safe(trial) and (
                best_expandable is None
                or trial.kp > best_expandable.kp + KP_DUPLICATE_TOLERANCE
                or trial.metrics.search_cost < best_expandable.metrics.search_cost
            ):
                best_expandable = trial

        stage_budgets = allocate_stage_iteration_budgets(args.kp_iterations)
        print(f"kp_stage_budgets: {stage_budgets}")
        current_stage_low = low
        current_stage_high = high
        history_start_index = 0
        stage_transition_count = 0

        for stage_index in range(AUTO_TUNE_MAX_STAGES):
            stage_budget = stage_budgets[min(stage_index, len(stage_budgets) - 1)]
            stage_history_limit = min(args.kp_iterations, history_start_index + stage_budget)
            stage_label = f"stage#{stage_index + 1}"

            while len(history) < stage_history_limit:
                midpoint = current_stage_low + (current_stage_high - current_stage_low) / 2.0
                stage_seed_candidates = [current_stage_low, midpoint, current_stage_high]
                if KP_BAYES_INITIAL_SAMPLE_COUNT > 3:
                    stage_seed_candidates.extend(
                        current_stage_low + (current_stage_high - current_stage_low) * index / max(1, KP_BAYES_INITIAL_SAMPLE_COUNT - 1)
                        for index in range(KP_BAYES_INITIAL_SAMPLE_COUNT)
                    )
                stage_seed_candidates = sorted(set(round(candidate, 6) for candidate in stage_seed_candidates))
                stage_progress = False
                for seed_kp in stage_seed_candidates:
                    if len(history) >= stage_history_limit:
                        break
                    seed_kp = float(seed_kp)
                    seed_key = round(seed_kp, 6)
                    if seed_key in evaluated_trials:
                        continue
                    print(
                        f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} seed kp={seed_kp:.6f}, interval=({current_stage_low:.6f}, {current_stage_high:.6f})"
                    )
                    seed_result = evaluate_kp_candidate(seed_kp, action_specs)
                    print_metrics(seed_result.metrics, prefix=f"kp={seed_kp:.6f} ")
                    print_trial_summary(seed_result, prefix=f"kp={seed_kp:.6f} ")
                    register_trial(seed_result)
                    stage_progress = True

                while len(history) < stage_history_limit:
                    if current_stage_high - current_stage_low < KP_INTERVAL_TOLERANCE:
                        print(
                            f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} interval converged ({current_stage_low:.6f}, {current_stage_high:.6f})"
                        )
                        break

                    observed_trials = sorted(
                        (
                            trial for trial in evaluated_trials.values()
                            if current_stage_low - KP_DUPLICATE_TOLERANCE <= trial.kp <= current_stage_high + KP_DUPLICATE_TOLERANCE
                        ),
                        key=lambda trial: trial.kp,
                    )
                    if len(observed_trials) < 2:
                        break
                    observed_kps = [trial.kp for trial in observed_trials]
                    observed_costs = [trial.metrics.search_cost for trial in observed_trials]
                    candidate_kp, candidate_ei = choose_next_kp_via_bayes(observed_kps, observed_costs, current_stage_low, current_stage_high)
                    candidate_key = round(candidate_kp, 6)
                    print(
                        f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} bayes kp={candidate_kp:.6f}, ei={candidate_ei:.6f}, interval=({current_stage_low:.6f}, {current_stage_high:.6f})"
                    )

                    if candidate_ei < KP_BAYES_EI_TOLERANCE and len(history) >= history_start_index + KP_MIN_ITERATIONS:
                        print(
                            f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} expected improvement too small, stop stage"
                        )
                        break

                    if candidate_key in evaluated_trials:
                        print(
                            f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} candidate already evaluated, stop stage"
                        )
                        break

                    candidate_result = evaluate_kp_candidate(candidate_kp, action_specs)
                    print_metrics(candidate_result.metrics, prefix=f"kp={candidate_kp:.6f} ")
                    print_trial_summary(candidate_result, prefix=f"kp={candidate_kp:.6f} ")
                    register_trial(candidate_result)
                    stage_progress = True

                if not stage_progress:
                    print(
                        f"kp_search[{len(history) + 1}/{args.kp_iterations}]: {stage_label} no new candidate, stop stage"
                    )
                    break
                if len(history) >= stage_history_limit:
                    break
                break

            transition_candidate = best_search if best_search is not None and best_search.metrics.stable_ok else best_expandable
            active_best = transition_candidate if transition_candidate is not None else best_search
            if active_best is not None:
                best_expandable_text = (
                    f"{best_expandable.kp:.6f}" if best_expandable is not None else "none"
                )
                transition_text = (
                    f"{transition_candidate.kp:.6f}" if transition_candidate is not None else "none"
                )
                print(
                    f"kp_stage_summary[{stage_label}]: best_search={best_search.kp:.6f} "
                    f"best_expandable={best_expandable_text} "
                    f"transition_candidate={transition_text}"
                )
            if active_best is None:
                break
            if stage_index >= AUTO_TUNE_MAX_STAGES - 1:
                break
            if len(history) >= args.kp_iterations:
                break
            if transition_candidate is None:
                break
            if not transition_candidate.metrics.stable_ok:
                break

            current_stage_low, current_stage_high = next_stage_bounds(transition_candidate.kp, hard_max, stage_index + 1)
            history_start_index = len(history)
            stage_transition_count += 1
            print(
                f"kp_stage_transition[{stage_transition_count}/{AUTO_TUNE_MAX_STAGES - 1}]: center kp={transition_candidate.kp:.6f} -> ({current_stage_low:.6f}, {current_stage_high:.6f})"
            )

        if best_search is None:
            raise RuntimeError("Kp auto-tuning produced no valid trials")

        selected_best = best_strict if best_strict is not None else best_search
        write_kp(args.kp_alias, args.kp_index, args.kp_subindex, selected_best.kp)
        print(f"kp_best: {selected_best.kp:.6f}")
        return AutoTuneResult(
            best_kp=selected_best.kp,
            history=history,
            original_kp=original_kp,
            baseline_result=baseline_result,
            best_result=selected_best,
            action_labels=[action_label for action_label, _, _ in action_specs],
            reset_position=reset_position,
        )

    print(
        f"joint_context: joint={JOINT_NAME} group={COMMAND_GROUP} "
        f"state_topic={args.state_topic} kp_alias={args.kp_alias}"
    )
    if args.setup_joint_positions:
        setup_text = ",".join(
            f"{joint_name}:{position:.6f}"
            for joint_name, position in ordered_joint_positions(args.setup_joint_positions)
        )
        print(f"setup_joints: {setup_text}")
    rclpy.init()
    node = ExecuteCommandClient(args.service, args.state_topic)
    kp_restore_value: Optional[float] = None
    return_code = 1
    try:
        original_kp: Optional[float] = None
        history: list[KpTrialResult] = []
        baseline_result: Optional[KpTrialResult] = None
        best_result: Optional[KpTrialResult] = None
        action_labels: list[str] = []
        reset_position: Optional[float] = None
        kp_trials_raw_csv_path: Optional[str] = None
        kp_trials_summary_csv_path: Optional[str] = None
        best_kp: Optional[float] = None
        should_save_single_plot = not args.no_plot
        base_req = MotionRequest(
            position=args.position if args.position is not None else 0.0,
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
            best_result = result.best_result
            action_labels = result.action_labels
            reset_position = result.reset_position
            print(f"kp_selected: {best_kp:.6f}")
            kp_trials_raw_csv_path, kp_trials_summary_csv_path = save_kp_trials_csvs(
                baseline_result,
                history,
                best_kp,
                args.kp_raw_csv_output or default_kp_trials_raw_csv_path(),
                args.kp_summary_csv_output or default_kp_trials_summary_csv_path(),
            )
            should_save_single_plot = False
        else:
            command, response, initial_position, tracker, metrics = run_single_motion_trial(base_req)
    except Exception as exc:
        print(f"Failed to move {JOINT_NAME}: {exc}", file=sys.stderr)
    else:
        if args.auto_tune_kp:
            print(f"kp_best: {best_kp:.6f}")
            print_metrics(best_result.metrics, prefix="kp_selected ")
            print_trial_summary(best_result, prefix="kp_selected ")
            print(f"kp_trials: {len(history)}")
            if kp_trials_raw_csv_path is not None:
                print(f"kp_trials_raw_csv: {kp_trials_raw_csv_path}")
            if kp_trials_summary_csv_path is not None:
                print(f"kp_trials_summary_csv: {kp_trials_summary_csv_path}")
            if baseline_result is not None and best_result is not None and not args.no_plot:
                plot_base_path = Path(args.plot_output or default_auto_tune_plot_path())
                for action_label in action_labels:
                    baseline_plot_trial = find_raw_trial(baseline_result, action_label, repeat_index=2)
                    best_plot_trial = find_raw_trial(best_result, action_label, repeat_index=2)
                    comparison_output = comparison_action_plot_path(str(plot_base_path), action_label)
                    if baseline_plot_trial is None or best_plot_trial is None:
                        print(
                            f"comparison_plot[{action_label}]: unavailable (missing repeat#2 raw trial)"
                        )
                        continue
                    try:
                        comparison_path = save_comparison_plot(
                            baseline_plot_trial,
                            best_plot_trial,
                            str(comparison_output),
                        )
                        print(f"comparison_plot[{action_label}]: {comparison_path}")
                    except Exception as exc:
                        print(f"comparison_plot[{action_label}]: unavailable ({exc})")
            return_code = 0
        else:
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

            if should_save_single_plot and tracker.command_time is not None:
                plot_output = args.plot_output or default_plot_path(base_req.position)
                try:
                    actual_plot_path = save_response_plot(tracker, plot_output, metrics.rmse)
                    print(f"response_plot: {actual_plot_path}")  # 打印图像路径，便于用户直接打开查看。
                except Exception as exc:
                    print(f"response_plot: unavailable ({exc})")  # 绘图失败时不影响主流程，只输出原因。
            return_code = 0 if response.success else 2  # 成功返回 0；服务有响应但执行失败时返回 2。
    finally:
        if args.auto_tune_kp and kp_restore_value is not None:  # 自动调参路径无论成功、失败还是绘图异常，最终都恢复原始 Kp。
            try:
                write_kp(args.kp_alias, args.kp_index, args.kp_subindex, kp_restore_value)
                print(f"kp_restored: {kp_restore_value:.6f}")
            except Exception as exc:
                print(f"kp_restore_failed: {exc}", file=sys.stderr)
        node.destroy_node()  # 正常结束前销毁 ROS 2 节点。
        rclpy.shutdown()  # 关闭 ROS 2 Python 运行时。
    return return_code


if __name__ == "__main__":  # 只有直接运行本文件时才进入这里；被 import 时不会自动执行。
    sys.exit(main())  # 执行主函数，并把返回值作为进程退出码。
