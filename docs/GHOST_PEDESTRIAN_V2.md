# 鬼探头 v2 —— 临界点离散化验证、Oracle 制动反事实与失效因果链（AutoDL）

## 目标

v1 把临界点定位在 24.25–24.5 m，并给出「物理空间充足、主因是制动时机」的**初步判别**。v2 解决三个遗留问题：

1. **P0** 验证 `Finalize` 后 trace 停止追加的修复；
2. **P1** 检验 24.25 m 与 24.5 m 的差异是否只是 evaluator 20 Hz 的**单 tick 离散化**，而非模型行为的真实阈值；
3. **P2** 用 **Oracle 制动反事实**从因果上确认「在正确时刻施加适度制动即可避免碰撞」，排除物理空间不足与「必须紧急制动」两种解释；
4. **P3** 建立统一时间线，并核查 TransFuser 是否存在可读取的感知输出，以界定「图像可见」与「模型识别到行人」的证据边界。

研究问题随之收敛为：**在遮挡行人突现情境下，端到端自动驾驶模型的安全行为边界在哪里、为何在此失效、如何高效定位该边界。** v2 仍不引入新变量（不改行人速度、遮挡车位置、光照、场景），不训练、不改 LEAD 核心代码。

## 方法

沿用 v0/v1 全部约束：Town01、晴天、seed 0、单次客户端 tick、场景内不调用 `load_world()`、触发距离按沿路线自车到横穿线距离定义。

| 项目 | 值 |
|---|---|
| 源码 | `carla-autodrive-validation/ghost_pedestrian/`（提交见下）|
| 场景新增字段 | `release_frame`、`actual_trigger_distance_m`、`visible_time_s`；`release_time_s` 保留 |
| Oracle 钩子 | `oracle_mode`：`immediate` / `visible` / `visible_delay`；`oracle_brake`（0.3–1.0）；`oracle_delay` |
| Oracle 路线 | 24 m（immediate / visible / visible+0.2 s / brake=0.3 / brake=0.5）、24.25 m（visible）、20 m（immediate / visible） |
| 临界区复跑 | 24.25 m、24.5 m 各再跑一次；24 m 再跑一次以获得完整字段 |
| 单次运行 | `run_ghost_test.sh <route.xml> 420`，CARLA 每次干净重启 |
| 汇总 | `analyze.py` → `results/summary.json`、`results/scan.csv`（新增 P1/P2 表）|

**Oracle 注入位置**：`scenario_manager._tick_scenario()` 的顺序是 `agent_wrapper() → apply_control(agent_action) → scenario_tree.tick_once() → world.tick()`。因此场景内 `OracleBrake` 在 agent 命令之后再次 `apply_control()` 即可覆盖下一物理步的控制，且 steer 保留 agent 输出。该行为挂在 `RecordGhostMetrics` 之前，使 trace 记录的是覆盖后的控制。制动指令是真实 CARLA `VehicleControl.brake`，不是假设的固定减速度。

## P0：`Finalize` 修复验证

对 `gp_24m_or_visible`：`trace.jsonl` 311 行，`summary.json.num_records` = 311 —— 完全一致。修复后不再出现 v1 中 24.5 m 运行 2088 行 trace / 311 条记录的情况，且各指标数值与预期一致。**P0 通过。**

## P1：临界点是单 tick 离散化，不是模型阈值

### 关键证据

| 运行 | 名义距离 | 释放时刻 t (s) | 释放帧 | 实际触发距离 (m) | 结果 |
|---|---|---|---|---|---|
| v1 `gp_24p5m` | 24.5 | 10.60 | — | — | 安全 |
| v1 `gp_24p25m` | 24.25 | **10.65** | — | — | **碰撞** |
| v2 `gp_24p5m`（复跑）| 24.5 | 10.55 | 1248 | 23.813 | 安全 |
| v2 `gp_24p25m`（复跑）| 24.25 | **10.55** | 1213 | 23.799 | **安全**（v1 为碰撞）|
| v2 `gp_24m`（复跑）| 24.0 | 10.60 | 1241 | 23.309 | 碰撞 |

1. v1 中 24.5 m 释放于 t=10.60，24.25 m 释放于 t=10.65 —— **相差 0.05 s，正好一个 20 Hz tick**（自车速度约 8 m/s，单 tick 沿路线约 0.4 m）。
2. v2 中同一个名义 24.25 m 复跑释放于 t=10.55 而变为**安全**；结果随释放落在哪个 tick 而翻转，而不是随名义距离单调变化。
3. 名义距离 ≠ 实际触发距离：24 m 各跑的实际值为 23.29–23.44 m，24.25 m 为 23.76–23.80 m，24.5 m 为 23.81 m。触发器按离散 tick 首次满足 `距离 ≤ 阈值` 时释放，产生 0.2–0.7 m 的系统性下冲与 tick 级抖动。

**结论**：0.25 m 的网格步长小于单 tick 对应的 0.4 m，相邻名义距离会落到同一或相邻 tick，因此 24.25–24.5 m 的「边界」是 evaluator 采样离散化产物，**不是端到端模型的安全阈值**。用户提出的离散化假设得到证实；按用户建议，不再继续细分到 0.01 m。

## P2：Oracle 制动反事实

所有 Oracle 组均**无碰撞**，且最小距离远大于 v1 碰撞组的 1.7–1.9 m：

| 运行 | 名义距离 | 模式 | 碰撞 | 最小距离 (m) | 3 s 制动面积 | 结果 |
|---|---|---|---|---|---|---|
| `gp_24m_or_immediate` | 24 | 释放即制动 | 否 | 19.69 | 2.95 | 安全 |
| `gp_24m_or_visible` | 24 | 首次几何可见制动 | 否 | 15.00 | 2.45 | 安全 |
| `gp_24m_or_delay020` | 24 | 可见后 +0.2 s | 否 | 12.43 | 2.10 | 安全 |
| `gp_24m_or_visible_b03` | 24 | 可见制动，brake=0.3 | 否 | 13.61 | 0.68 | 安全 |
| `gp_24m_or_visible_b05` | 24 | 可见制动，brake=0.5 | 否 | 13.60 | 1.12 | 安全 |
| `gp_24p25m_or_visible` | 24.25 | 可见制动 | 否 | 13.91 | 2.40 | 安全 |
| `gp_20m_or_immediate` | 20 | 释放即制动 | 否 | 16.35 | 2.95 | 安全 |
| `gp_20m_or_visible` | 20 | 可见制动 | 否 | 10.76 | 2.40 | 安全 |

三个关键判断：

1. **物理空间不是瓶颈**：在 20 m 这种 v1 必撞的距离，释放即制动仍留出 16.35 m 间隙。这与 v1 的物理余量分析一致（24 m 时余量约 19.6 m，所需停车距离仅 3.8 m）。
2. **不需要紧急制动**：仅在首次几何可见时施加 **30%** 制动即可避免碰撞（最小距离 13.6 m），且 +0.2 s 延迟仍安全。因此「LEAD 必须做出剧烈制动」不是必要条件；问题在于制动是否**及时且持续**。
3. **因果上定位到控制决策**：在完全相同的场景、自车与行人状态下，仅把 LEAD 控制替换为「可见后适度持续制动」就消除碰撞，且制动指令是 CARLA 可实现的真实命令。因此 v1 的碰撞不能归因于物理不可能，也不能归因于「感知不可用」（几何可见时刻稳定在释放后 0.70 s，Oracle 的可行介入正以此为准）。

注意：Oracle 是**充分性**反事实（证明及时制动足以避险），不是对 LEAD 内部感知状态的直接测量；它不单独证明「LEAD 没看到行人」。

## P3：统一时间线与感知输出证据边界

同一名义 24 m、同样释放于 t=10.60 的两次运行，相对释放时刻的因果链：

| 事件 | LEAD 原控制（`gp_24m`）| Oracle 可见制动（`gp_24m_or_visible`）|
|---|---|---|
| 释放（几何遮挡结束前）| t=0（帧 1241，实际触发 23.309 m）| t=0（帧 1129，实际触发 23.439 m）|
| 行人几何可见 | +0.70 s | +0.70 s |
| 首次制动 | **+1.20 s**（可见后 0.50 s）| +0.55 s（LEAD 自身先有制动），Oracle 于 +0.75 s 覆盖为持续制动 |
| 行人进入自车车道 | +1.35 s | +1.35 s |
| 自车前保险杠到达横穿线 | **+2.95 s** | 未到达（已减速停车）|
| 结果 | **碰撞**，最小距离 1.83 m | 安全，最小距离 15.00 m |

LEAD 在可见后约 0.5 s 才出现持续制动，而 Oracle 在可见后约 0.05 s 即施加持续制动；两者相差约 0.45 s，正好决定碰撞与否。结合 v1 的安全组（把车速降到 0–3.1 m/s）与碰撞组（最低仍 5.0–6.9 m/s），失效机理最一致的解释是：**感知输入可用、物理空间充足，但 LEAD 的制动决策在幅度/持续性上不足，未能把到达时刻推迟约 0.1–0.2 s。**

### 感知输出核查（证据边界）

- `checkpoints/transfuser/config.yaml` 中 `detect_boxes: true`、`use_bev_semantic: true` —— 模型每步都会计算 `bounding_box` 与 `bev_semantic` 辅助输出。
- `Prediction` 数据类确实包含 `semantic`、`depth`、`bounding_box`、`bev_semantic` 等字段，`TransfuserAgent.compute_control()` 接收完整 `Prediction`，并通过 `AgentPrediction(prediction=prediction, ...)` 保留在 `self.agent_prediction.prediction` 中，**无需改模型即可挂钩记录**。
- 但实际控制仅由 `prediction.route`、`prediction.future_waypoints`、`prediction.target_speed_scalar/distribution` 经 `WaypointTracker`/`PathSpeedTracker` 产生；检测/BEV 语义头是训练辅助任务，**不直接驱动 steer/throttle/brake**。

因此当前只能断言「行人在 t_rel=0.70 s 进入模型图像输入且几何可见」，**不能**断言「模型内部检测到了行人」。要严格区分「未及时感知」与「决策制动不足」，仍需在 `compute_control()` 处记录 `prediction.bounding_box`/`bev_semantic` 中该行人的框与置信度，得到「首次检测到」时刻，与 0.70 s 对比。这是下一步 P3 深挖的具体挂钩点，也是诚实结论所要求的证据边界。

## 清理与已知数据瑕疵

按确认仅清理临时文件：Oracle/复跑产生的 `/tmp/gp_*.out`、`/tmp/or24v.out` 等临时评估输出可删除。完整保留：所有 `*.eval.log`、`trace.jsonl`、`summary.json`、`results/`、`routes/`、场景源码、`frames/` 与全部 Git 提交。

已知瑕疵：`gp_24p5m` 的旧 trace 仍是修复前的 2088 行（保留原样供复核），其指标不受影响；v2 新跑的 trace 行数均与 `num_records` 一致。`release_frame` 是 CARLA 服务器绝对帧计数，跨重启不可直接比较，只有同一次运行内的帧差或 `release_time_s` 有跨运行可比性。LEAD 侧 `load_world()` 跳过补丁与 `.orig_autodl` 备份保持原状，未覆盖无关输出/检查点。

## 结论与后续（P4/P5）

1. **边界定位**：24.25–24.5 m 的表观临界点是 20 Hz tick 离散化产物；不应再细分网格，应改用带 tick 抖动的区间估计（当前实际触发距离抖动约 0.15–0.4 m）。
2. **失效机理（当前最佳解释）**：感知输入及时可用、物理空间充足；LEAD 制动决策在幅度/持续性上不足，未能把到达时刻推迟约 0.1–0.2 s。Oracle 30% 制动即可避险，从因果上排除「物理空间不足」与「必须紧急制动」。
3. **证据边界**：尚未证明模型内部是否检测到行人；需记录 TransFuser 检测输出。
4. **P4 重复性**：临界区每个距离增加到 3–5 次，用实际触发距离而非名义距离做区间统计，给出带置信度的边界。
5. **P5 扩展变量**：固定触发距离后改行人速度（1.0 / 2.0 m/s）或遮挡车位置，检验边界移动与机理普适性。
