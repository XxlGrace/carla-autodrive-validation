# 鬼探头 v3 —— 释放时序修正、条件—结果分布、模型内部输出与主动边界搜索（AutoDL）

> **后续版本**：[`GHOST_PEDESTRIAN_V4.md`](GHOST_PEDESTRIAN_V4.md) 重建了场景本身（城市路段、路侧停车遮挡、物理行人、LEAD 前视相机视角录制、逐帧遮挡实测）。v4 的道路/遮挡/相机都变了，**v3 的临界距离不能迁移到 v4**；本文保留为机制验证的历史记录。

## 目标

v2 把 24.25–24.5 m 的表观临界点归因于 20 Hz tick 离散化，并用 Oracle 制动证明物理空间充足。v3 处理三类遗留问题：

1. **P0.5** 修正「实际触发距离」的测量时序（`RecordGhostMetrics` 先于 `ReleasePedestrian` 执行，导致距离采样晚一个物理 tick），同时记录 Oracle **实际生效**的帧/时间而非计划时间。
2. **P4** 把实验单位从「名义距离」改为「真实释放状态」`s_release = (d_actual, v_ego, t_release, walker_state, ...)`；在 24 / 24.25 / 24.5 / 25 m 各做 5 次独立重复，区分稳定失效与运行波动。
3. **P3.5** 在 LEAD 推理流程中记录 `bounding_box`、`bev_semantic`、`target_speed`、`future_waypoints` 与原始控制，把「决策不足」分解为可验证的感知/规划/控制问题。
4. **P5** 只增加一个变量——行人横穿速度——给出二维安全结果图，检验非单调性。
5. **P6** 小规模主动边界搜索：对比「仅几何参数」基线与「几何 + 实测响应信息」的边界定位效率。

v3 不改变场景几何、不训练模型、不引入天气/光照/道路结构变量。

## 方法与工程约束

沿用 v0/v1/v2 全部约束：Town01、晴天、seed 0、单客户端 tick、场景内不 `load_world()`、无第二客户端 tick。

| 项目 | 值 |
|---|---|
| 场景源码 | `ghost_pedestrian/scenarios/ghost_pedestrian.py`（v3 修改见下） |
| 路线生成 | `scripts/gen_routes.py`，共 78 条 XML |
| 运行器 | `scripts/run_batch.sh <list> <workers> [timeout]`，每 worker 独立 CARLA 与 traffic-manager 端口，每条路线前干净重启 |
| 汇总 | `scripts/analyze_v3.py` → `results/p4_release_states.csv`、`p35_timeline.json`、`p5_map.csv` |
| 主动搜索 | `scripts/p6_active_search.py` → `results/p6_active_search.json` |
| 文档生成 | `scripts/gen_v3_doc.py`（本文件由它生成） |
| Demo | `scripts/make_demos.sh` → `demos/*.mp4`，说明见 `demos/README.md` |

**并行度**：RTX 4090 上 3 个 CARLA 实例同时运行会出现 SIGSEGV；所有正式实验均以 1 worker 顺序执行。偶发的陈旧 CARLA 占用出生点会导致 `Couldn't spawn the ego vehicle`（空 summary，`num_records=0`），这类运行已识别并重跑。

### P0.5 时序修正

行为树根为 `Parallel(SUCCESS_ON_ONE)`，子节点顺序：`OracleBrake` → `RecordGhostMetrics` → `seq`。同一 tick 内 `RecordGhostMetrics` 先执行、`ReleasePedestrian` 后执行：原实现在 `RecordGhostMetrics` 首次看到 `released=True` 时记录的距离，已是**释放后一个物理步**的距离。

修正：把释放瞬间状态采集移入 `ReleasePedestrian.update()`，在设置 `released=True` 的同一调用中读取自车坐标、速度、快照 frame、到横穿点距离与行人坐标；原「下一 tick 距离」另存为 `first_post_release_distance_m`。`OracleBrake` 记录 `oracle_planned_start_time_s` 与**实际**覆盖控制的 `oracle_first_active_time_s` / `oracle_first_active_frame`，并在 trace 中增加 `oracle_active` 标志。summary 新增 `trigger_error_m`、`release_speed_mps`、`release_ego_x/y`、`release_walker_x/y`、`walker_speed_mps`。

## P0.5 结果：一个 tick 的实测偏移

| 运行 | 名义距离 (m) | 释放瞬间 (m) | 下一采样 (m) | 差值 (m) |
|---|---|---|---|---|
| p4_24m_r1 | 24.00 | 23.666 | 23.306 | **0.360** |
| p4_24m_r2 | 24.00 | 23.713 | 23.305 | **0.408** |
| p4_24m_r3 | 24.00 | 23.934 | 23.534 | **0.400** |
| p4_24m_r4 | 24.00 | 23.713 | 23.324 | **0.389** |

差值范围 0.360–0.441 m（均值 0.390 m），与「约 7–8 m/s × 0.05 s」一致，直接证实测量时序偏移。v2 报告的「实际触发距离」应理解为**释放后一 tick 的距离**，约比真实释放距离小 0.37–0.41 m；v2 中 24 m 名义点的实际释放距离应上修约 0.4 m。

Oracle 的 `immediate` 模式实际生效晚于计划时间一个行为树 tick（`OracleBrake` 在 `ReleasePedestrian` 之前执行），现已记录在案。碰撞与 Oracle 避险结论不受影响（两者用同一释放机制），但精细边界解释须用修正后的数值。

## P4：条件—结果分布

每个名义距离 5 次独立运行；**实验单位是实测释放状态** `s_release`，名义距离只是控制输入的标签。

| 名义距离 (m) | n | 实测释放距离均值±sd (m) | 释放速度均值±sd (m/s) | 碰撞 | 下一采样距离均值 (m) |
|---|---|---|---|---|---|
| 24.0 | 5 | 23.77 ± 0.095 | 7.77 ± 0.349 | 5/5 | 23.38 |
| 24.25 | 5 | 24.04 ± 0.146 | 7.94 ± 0.554 | 2/5 | 23.64 |
| 24.5 | 5 | 24.16 ± 0.029 | 7.68 ± 0.170 | 0/5 | 23.78 |
| 25.0 | 5 | 24.78 ± 0.143 | 7.88 ± 0.118 | 0/5 | 24.39 |

### 混合结果的关键证据

- 名义 **24.25 m** 出现混合结果：安全 3 次（实测释放 24.156 m，速度 7.66 m/s）；碰撞 2 次（实测释放 23.861 m，速度 8.36 m/s）。**同一名义标签下的结果差异由实测释放状态解释**，而非随机噪声。

5 次重复是**初步稳定性证据**，不足以给出精确概率或置信区间；它足以说明「名义距离相同 → 结果可以不同，且差异与实测释放状态同向」。完整明细见 `ghost_pedestrian/results/p4_release_states.csv`。

## P3.5：LEAD 内部时间线

新增 env-gated 只读钩子（`GP_PRED_LOG`，补丁存于 `ghost_pedestrian/lead_patches/gp_pred_hook.patch`）：未设置环境变量时每步仅一次字典查询，行为与原来完全一致。记录字段：

- 感知辅助输出：`bounding_box`（类别 1 = WALKER 的框与置信度）、`bev_semantic`（类别 5 = WALKER 的最大概率与高置信格数）、`semantic`（类别 4 = PEDESTRIAN）。
- 规划输出：`target_speed_scalar`、经 brake threshold 处理后的 `target_speed`、`future_waypoints`、`route`。
- 控制分解：`waypoints_brake`、`target_speed_brake`、`target_speed_throttle` 与最终 `steer/throttle/brake`。
- 对齐键：LEAD `step`、CARLA world snapshot `frame` 与 `elapsed_seconds`。

**重要限定**：几何可见性由场景额外安装的相机计算，不能等同于 LEAD 的实际输入传感器。此处以 LEAD 进程内、该步模型实际消费的相机/LiDAR 对应的 frame ID 为准。

判定规则：

- **情况 A**：辅助检测显著晚于几何可见（+0.70 s）→ 模型表征行人信息的过程值得调查，但不能直接认定控制失败完全由检测造成。
- **情况 B**：辅助检测已出现，但预测目标速度未及时下降 → 危险信息未充分反映到规划输出。
- **情况 C**：目标速度及时下降但实际制动仍晚 → 问题在轨迹跟踪/控制执行时序。

### `p35_lead_24m`

- 模型步数：413；首次辅助检测（walker box 或 BEV 概率 > 0.15）：**— s**；目标速度减半：**0.827 s**（相对释放时刻）。

| 目标 t_rel (s) | 实际 t_rel | frame | 遮挡 | walker_y | boxW_max | bevW_max | tgtSpd_post | brake | 控制 brake |
|---|---|---|---|---|---|---|---|---|---|
| 0.50 | 0.477 | 713 | True | 207.999 | 0.0000 | 0.0000 | 8.490 | 0.000 | 0.000 |
| 0.70 | 0.677 | 717 | False | 207.999 | 0.0000 | 0.0001 | 8.740 | 0.000 | 0.000 |
| 0.90 | 0.877 | 721 | False | 207.999 | 0.0000 | 0.0001 | 0.000 | 0.000 | 1.000 |
| 1.20 | 1.177 | 727 | False | 207.999 | 0.0000 | 0.0001 | 0.000 | 0.000 | 1.000 |
| 1.50 | 1.477 | 733 | False | 207.915 | 0.0000 | 0.0001 | 0.000 | 0.000 | 1.000 |

### `p35_or_24m`

- 模型步数：625；首次辅助检测（walker box 或 BEV 概率 > 0.15）：**10.839 s**；目标速度减半：**0.289 s**（相对释放时刻）。

| 目标 t_rel (s) | 实际 t_rel | frame | 遮挡 | walker_y | boxW_max | bevW_max | tgtSpd_post | brake | 控制 brake |
|---|---|---|---|---|---|---|---|---|---|
| 0.50 | 0.489 | 715 | True | 207.999 | 0.0000 | 0.0000 | 0.000 | 0.000 | 1.000 |
| 0.70 | 0.689 | 719 | False | 207.999 | 0.0000 | 0.0000 | 0.000 | 0.000 | 1.000 |
| 0.90 | 0.889 | 723 | False | 207.999 | 0.0000 | 0.0001 | 0.000 | 1.000 | 1.000 |
| 1.20 | 1.189 | 729 | False | 207.999 | 0.0000 | 0.0000 | 0.000 | 1.000 | 1.000 |
| 1.50 | 1.489 | 735 | False | 207.915 | 0.0000 | 0.0001 | 0.667 | 1.000 | 0.000 |

**解读（基于已生成的数据）**：几何首次可见在 +0.70 s。在 LEAD 碰撞运行中，辅助检测头在决定胜负的 0–1.5 s 窗口内对行人的响应几乎为零（box WALKER 最大置信度 0.0，BEV WALKER 最大概率约 0.0001），显著检测直到约 +10.8 s（行人早已穿过）才出现；但目标速度在 +0.83 s 已降为 0，模型自身制动在 +0.90 s 达到 1.0。因此该失效**不是**「完全没有减速」，而是「减速在物理上来得太晚」；同时辅助感知头未能及时表征行人，属于情况 A/B 的混合，需进一步用 LEAD 实际输入传感器的 frame 对齐来区分检测与规划各自的贡献。

完整原始序列见各运行的 `ghost_runs/<run>/pred.jsonl`。

## P5：行人速度的二维结果图

| 速度 (m/s) \ 距离 (m) | 20 | 22 | 24 | 26 |
|---|---|---|---|---|
| 1 | COLL | COLL | COLL | COLL |
| 1.5 | COLL | COLL | SAFE | SAFE |
| 2 | SAFE | SAFE | SAFE | SAFE |

最小距离 (m)：

| 速度 (m/s) \ 距离 (m) | 20 | 22 | 24 | 26 |
|---|---|---|---|---|
| 1 | 0.98 | 0.90 | 0.16 | 0.42 |
| 1.5 | 1.17 | 1.25 | 2.01 | 2.15 |
| 2 | 2.08 | 2.10 | 2.12 | 2.14 |

- 速度 1.0 m/s：碰撞序列 `1111`，符号翻转 0 次。
- 速度 1.5 m/s：碰撞序列 `1100`，符号翻转 1 次（存在非单调性）。
- 速度 2.0 m/s：碰撞序列 `0000`，符号翻转 0 次。

**物理解读**：行人速度决定其占据自车车道的时长。1.0 m/s 横穿最慢，即使 26 m 释放仍碰撞；2.0 m/s 最快清空车道，20 m 释放已安全。用 P4 的有效时间裕度可统一解释：`arrival ≈ (d_actual − 2.45)/v_ego`，`clear ≈ 4.4/v_walker`。因此**触发距离本身不是充分的安全判据**，「距离 × 行人速度 × 释放瞬间自车速度」的联合状态才是。

完整明细见 `ghost_pedestrian/results/p5_map.csv`。

## P6：主动边界搜索

- 查询预算：每策略 6 次 CARLA 查询。
- 全部数据拟合的参考边界：22.750 m。

| 策略 | 查询点 | 找到的失效 | 最终边界 (m) | 与参考误差 (m) |
|---|---|---|---|---|
| baseline | p6_s1p5_d22p5m(collision), p6_s1p5_d23p5m(collision), p6_s1p5_d24p25m(safe), p6_s1p0_d26p5m(collision), p6_s1p5_d21p5m(collision), p6_s2p0_d20p5m(safe) | 4 | 23.125 | 0.375 |
| informed | p6_s1p5_d21p5m(collision), p6_s1p5_d23p5m(collision), p6_s1p5_d22p5m(collision), p6_s1p5_d24p25m(safe), p6_s2p0_d20p5m(safe), p6_s1p5_d25p5m(safe) | 3 | 23.000 | 0.250 |

**解读（需谨慎）**：informed 策略的边界定位误差为 0.25 m，小于几何基线的 0.375 m（约小 33%），但失效命中数反而更少（3 次 vs 4 次）。此外两策略前 5 次查询高度重合——因为它们从同一初始数据集出发，未跑候选的响应特征由 kNN 补全，导致选点相近。因此**不能断言统计显著的优势**。更有价值的结论与 P4 一致：把实验单位从名义几何参数改成实测释放状态（真实触发时序 + 模型响应）后，失效边界的定位更稳定；这可以作为论文入口，但需要在更大预算下复验。

## Demo

`demos/` 下提供 5 个 MP4：遮挡车控制组、25 m 安全、24 m 碰撞、Oracle 制动避险，以及四合一对比。视频由场景监控相机帧合成，**不是 LEAD 的输入传感器画面**。运行 `scripts/make_demos.sh` 可重建。

| 文件 | 运行 | 内容 |
|---|---|---|
| `demo_occluder_only.mp4` | `occluder_only` | 仅有遮挡车，无行人；自车正常通过（控制组） |
| `demo_safe_25m.mp4` | `gp_25m` | 25 m 释放；LEAD 制动让行，无碰撞 |
| `demo_collision_24m.mp4` | `gp_24m` | 24 m 释放；LEAD 到达过早，右前角接触 |
| `demo_oracle_brake.mp4` | `gp_24m_or_visible` | 同样 24 m 场景，首次可见时外部全制动；无碰撞 |
| `demo_comparison.mp4` | 2×2 网格 | 四段各取前 6 秒并排对比 |

阅读顺序：遮挡车 → 25 m 安全 → 24 m 碰撞 → Oracle 制动。关键是最后两段：场景与释放状态相同，仅控制不同，结果不同。

## 清理与提交

按确认仅清理临时文件（`/tmp/p4batch.out`、`/tmp/gpbatch/*`、`/tmp/p6_*.out`、`/tmp/gp_demo_check.png` 等）。完整保留 `*.eval.log`、`trace.jsonl`、`summary.json`、`results/`、`routes/`、场景源码、LEAD 补丁、`frames/`、`demos/` 与全部 Git 提交。LEAD 侧 `load_world()` 跳过补丁及 `.orig_autodl` 备份保持原状。

