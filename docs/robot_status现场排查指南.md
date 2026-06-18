# `/protocol/robot_status` 现场排查指南

> 状态：`archive`
> 
> 主入口请优先使用：`故障排查.md` 与 `对外接口文档.md`（状态判据章节）。

本文用于现场快速判断机器人是否处于以下异常状态：

- 迷路 / 定位丢失
- 卡住
- 急停

重点依据：

- `/protocol/robot_status`

辅助依据：

- `/protocol/mission_state`
- `/protocol/task_state`
- `/protocol/robot_alarm`

## 1. 先看什么

现场排查时，先同时打开这 4 个 topic：

```bash
ros2 topic echo /protocol/robot_status --full-length
```

```bash
ros2 topic echo /protocol/mission_state --full-length
```

```bash
ros2 topic echo /protocol/task_state --full-length
```

```bash
ros2 topic echo /protocol/robot_alarm --full-length
```

如果 `/protocol/robot_status` 没有持续刷新，先请求机器人主动上报：

```bash
ros2 run chassis_move protocol_cli -- request-status path --report-duration-sec 300
```

## 2. `robot_status` 里最关键的字段

当前协议里，`/protocol/robot_status` 的核心字段主要分布在以下对象中：

- `status_msg`
- `pose_msg`
- `speed_msg`
- `mission_msg`

### 2.1 `status_msg`

最关键字段：

- `lost`
  是否定位丢失
- `lost_reason`
  定位丢失原因
- `stuck`
  是否卡住
- `stuck_reason`
  卡住原因
- `estop`
  是否急停
- `estop_reason`
  急停原因
- `operation_mode`
  当前模式：`AUTOMATIC`、`SEMIAUTOMATIC`、`MANUAL`、`SERVICE`、`TEST`
- `work_status`
  当前工作状态，如 `IDLE`、`BUSY`、`FAIL`、`INTERRUPT`

### 2.2 `pose_msg`

最关键字段：

- `px`
  当前地图坐标 x(mm)
- `py`
  当前地图坐标 y(mm)
- `pt`
  当前地图朝向 deg
- `score`
  位姿置信度，范围 `0~1000`

### 2.3 `speed_msg`

最关键字段：

- `vx`
  x 方向速度 mm/s
- `vy`
  y 方向速度 mm/s
- `vt`
  角速度 deg/s

### 2.4 `mission_msg`

最关键字段：

- `mission_id`
- `mission_name`
- `mission_state`
- `task_id`
- `task_cmd`
- `task_state`

## 3. 如何判断“迷路 / 定位丢失”

### 3.1 最直接判据

只要看到：

- `status_msg.lost == true`

就可以直接判为：

- **机器人定位丢失**
- **现场可直接归类为“迷路”**

如果同时还有：

- `status_msg.lost_reason`

则优先记录原因原文。

### 3.2 强怀疑判据

即使 `lost` 还没变成 `true`，如果出现以下组合，也应判为“强怀疑迷路”：

- 正在执行导航任务，`mission_msg.task_cmd == "goto"`
- `pose_msg.px/py/pt` 跳变明显，不符合实际运动
- `pose_msg.score` 持续偏低或剧烈波动
- 机器人现场明显偏离预期位置
- `/protocol/mission_state` 长时间不收敛，甚至进入 `FAILED`

### 3.3 典型现象

常见迷路表现：

- 机器人原地小范围乱转
- 地图上位置突然跳到别处
- 任务状态还在 `RUNNING`，但运动方向明显不合理
- `task_cmd` 是 `goto`，但长时间到不了目标点

### 3.4 建议动作

如果确认迷路：

1. 先记录当前 `lost`、`lost_reason`、`score`、`mission_state`
2. 立即停止继续派发导航任务
3. 必要时执行停止指令
4. 视现场流程决定是否人工接管或重定位

停止指令：

```bash
ros2 run chassis_move protocol_cli -- stop
```

## 4. 如何判断“卡住”

### 4.1 最直接判据

只要看到：

- `status_msg.stuck == true`

就可以直接判为：

- **机器人卡住**

如果有：

- `status_msg.stuck_reason`

优先记录原因。

### 4.2 辅助判据

即使 `stuck` 还没置位，如果出现以下组合，也要高度怀疑卡住：

- 当前任务在执行 `move` 或 `goto`
- `speed_msg.vx/vy/vt` 显示机器人有运动尝试
- 但 `pose_msg.px/py/pt` 长时间几乎不变化
- `/protocol/task_state` 一直停留在同一任务状态
- 现场观察机器人被障碍物挡住、顶住、陷住

### 4.3 典型现象

常见卡住表现：

- 电机在发力，但车体基本不动
- 原地蹭动或轻微抖动
- 任务一直不结束
- 最后可能转成 `FAILED`

### 4.4 建议动作

如果确认卡住：

1. 先记录 `stuck`、`stuck_reason`、当前 `task_cmd`
2. 确认是否有外部障碍物
3. 先停止当前动作
4. 排除障碍后再决定重新发任务还是人工接管

## 5. 如何判断“急停”

### 5.1 最直接判据

只要看到：

- `status_msg.estop == true`

就可以直接判为：

- **机器人已处于急停状态**

如果有：

- `status_msg.estop_reason`

优先记录原因。

### 5.2 辅助现象

常见伴随现象：

- 速度迅速归零：`vx == 0`、`vy == 0`、`vt == 0`
- 当前任务无法继续推进
- 机器人模式可能切到 `MANUAL`
- 文档语义上，调度/就绪模式下触发 stop 后可能进入手操模式

### 5.3 建议动作

如果确认急停：

1. 先不要继续发导航任务
2. 记录 `estop_reason`
3. 检查是否为人为触发、硬件按钮触发或安全传感器触发
4. 确认解除急停条件后，再恢复模式和任务流程

## 6. 快速判断表

### 6.1 迷路

满足以下任一条，可直接或基本判定迷路：

- `status_msg.lost == true`
- `lost == false`，但位姿跳变严重且导航任务明显异常

优先看：

- `status_msg.lost`
- `status_msg.lost_reason`
- `pose_msg.score`
- `mission_msg.task_cmd`
- `/protocol/mission_state`

### 6.2 卡住

满足以下任一条，可直接或基本判定卡住：

- `status_msg.stuck == true`
- 有运动任务，但位姿长时间不变且现场有阻挡

优先看：

- `status_msg.stuck`
- `status_msg.stuck_reason`
- `speed_msg`
- `pose_msg`
- `task_cmd`

### 6.3 急停

满足以下任一条，可直接判定急停：

- `status_msg.estop == true`

优先看：

- `status_msg.estop`
- `status_msg.estop_reason`
- `speed_msg`
- `operation_mode`

## 7. 推荐排查顺序

建议现场按以下顺序判断：

1. 先看 `status_msg.estop`
   如果为 `true`，先按急停处理
2. 再看 `status_msg.lost`
   如果为 `true`，直接按迷路/定位丢失处理
3. 再看 `status_msg.stuck`
   如果为 `true`，按卡住处理
4. 再结合 `pose_msg.score`、`speed_msg`、`mission_msg.task_cmd`
   判断是否属于“尚未置位但已经明显异常”的情况
5. 最后结合 `/protocol/robot_alarm`
   看是否有更明确的报警说明

## 8. 最小现场结论模板

建议现场记录时直接写成下面格式：

```text
时间：
机器人ID：
当前任务：
当前子任务：
operation_mode：
work_status：
lost / lost_reason：
stuck / stuck_reason：
estop / estop_reason：
pose(px, py, pt, score)：
speed(vx, vy, vt)：
alarm：
现场现象：
初步结论：迷路 / 卡住 / 急停 / 其他
```

## 9. 当前实现边界

需要注意：

- 当前工作区已经能接收并转发这些状态字段
- 但还没有一个专门节点自动把 `lost/stuck/estop` 提炼成高层状态结论
- 因此现在的判断仍主要依赖人工观察 topic 内容

如果后续要进一步自动化，建议新增一个状态聚合节点，专门输出：

- `is_lost`
- `is_stuck`
- `is_estop`
- `fault_summary`
