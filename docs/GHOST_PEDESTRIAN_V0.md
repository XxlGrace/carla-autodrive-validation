# 鬼探头（Ghost Pedestrian）v0 —— LEAD 闭环安全测试

## 目标与范围

在已验证可用的 LEAD 闭环评估之上，增加一个**最小、可控、可复现**的鬼探头场景：行人被路侧停车完全遮挡，当自车接近交叉口到设定距离时突然横穿自车行驶轨迹。通过扫描「触发距离」寻找 LEAD 从安全避让到碰撞的临界点。

v0 明确不做：昼夜/天气变化、复杂场景、重新训练模型、危险分位数 / RL / 生成式方法。LEAD 驾驶代码与 evaluator 核心逻辑均未改动，场景作为独立模块通过 XML 自动发现接入。

## 组件与路径

| 项目 | 值 |
|---|---|
| 源码（source of truth） | `/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/` |
| 场景模块 | `scenarios/ghost_pedestrian.py`（类 `GhostPedestrian(BasicScenario)`）|
| 安装目标 | `third_party/lead/3rd_party/leaderboard/standard/scenario_runner/srunner/scenarios/ghost_pedestrian.py` |
| 安装脚本 | `scripts/install.sh`（仅复制文件，不动 LEAD 核心；自动备份旧副本）|
| 路线生成 | `scripts/gen_routes.py` → `routes/{baseline,occluder_only,gp_25m,gp_15m,gp_9m}.xml` |
| 单条运行 | `scripts/run_ghost_test.sh <route.xml> [timeout]` |
| 指标汇总 | `scripts/analyze.py` → `results/summary.json` |
| 原始证据 | `/root/autodl-tmp/ghost_runs/<run>/{trace.jsonl,summary.json,frames/*.png,<run>.eval.log}` |

## 几何设计（Town01，固定晴天）

路线：沿 `x≈396.3` 由北向南直行，`y=300 → 150`。触发点 `y=260`，横穿点 `y=208`（`crossing_ahead=52m`）。

| 元素 | 位置 | 说明 |
|---|---|---|
| 自车车道中心 | `x≈396.37` | LEAD 实际轨迹 |
| 遮挡车 | `(399.08, 213.00)`，yaw=-90° | `vehicle.lincoln.mkz_2020`，`simulate_physics=False`，置于路肩，不侵入自车车道 |
| 行人起点 | `(399.48, 208.00)` | 位于遮挡车正后方（自车视角），初始被完全遮挡 |
| 行人终点 | `(394.28, 208.00)` | 由东向西横穿，速度 1.5 m/s，全程 5.2 m，约 3.47 s |
| 触发距离 | 沿路线自车到横穿点的纵向距离（25 / 15 / 9 m）| 不是实时欧氏距离 |

行人从起点到终点必然穿过自车轨迹 `x≈396.37`，已由逐帧 `walker_x` 与 `ego_x` 验证（三次 GP 运行均为 `walker_crossed_ego_path=True`）。

## 如何运行

```bash
# 1) 生成路线（几何参数改动后需重跑）
python3 /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scripts/gen_routes.py

# 2) 安装场景模块（自动发现，无需改 LEAD 核心）
bash /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scripts/install.sh

# 3) 逐条评估（脚本会先干净重启 CARLA，再跑 evaluator）
for R in baseline occluder_only gp_25m gp_15m gp_9m; do
  bash /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scripts/run_ghost_test.sh ${R}.xml 420
done

# 4) 汇总
python3 /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scripts/analyze.py
```

环境变量、无头 Vulkan ICD、地图定图、`NUMBA_NUM_THREADS=24` 等前提与 `docs/LEAD_EVAL.md` 完全相同，此处不再重复。

## 实测结果（2026-11-22）

控制组与扫描结果（触发距离 = 自车到横穿点纵向距离）：

| 运行 | 触发距离 | 碰撞行人 | 最小距离 (m) | 制动滞后释放 (s) | 释放时车速 (m/s) | 释放时 TTC (s) | 释放前遮挡率 | 结果 |
|---|---|---|---|---|---|---|---|---|
| Baseline（无场景）| — | 否 | — | — | — | — | — | 控制组，正常通过 |
| Occluder Only | — | 否 | — | 未释放 | — | — | — | 控制组，正常通过 |
| GP 25 m | 25 | 否 | 2.112 | 1.45 | 8.05 | 3.11 | 0.754 | **安全避让** |
| GP 15 m | 15 | 是 | 0.787 | 0.10 | 8.35 | 1.80 | 0.812 | **碰撞** |
| GP 9 m | 9 | 是 | 1.396 | 2.25 | 8.03 | 1.12 | 0.832 | **碰撞** |

关键发现：

1. **临界点位于 25 m 与 15 m 之间**：25 m 时 LEAD 有约 3.1 s TTC，制动后以 2.11 m 最小距离通过；15 m 时 TTC 仅 1.8 s，虽几乎立刻制动仍发生接触（最小距离 0.79 m）。
2. **9 m 时制动晚于碰撞**：碰撞发生在释放后约 1.25 s，而首次明显制动在释放后 2.25 s —— 即在本次运行中 LEAD 是**撞上之后才制动**，说明极近距离下感知—决策链路来不及响应。
3. 三个 GP 运行的释放前遮挡率 0.75–0.83，证明行人被遮挡车真实遮挡；剩余样本为自车已经越过遮挡车、行人即将/已经出现，属正常。
4. 25 m 运行的 leaderboard 判定为 `MinSpeedTest FAILURE (97.89%)`，`RouteCompletionTest=100%`、`CollisionTest=0`。该 MinSpeedTest 失败由为避让行人而减速造成，不是场景 bug；评估「安全性」以本模块的碰撞/最小距离为准。

## 搭建过程中遇到的困难（重点记录）

以下按出现顺序记录在案，供后续复用/扩展参考。

### 1. `load_world()` 会 SIGSEGV（沿用既有约束）
场景本身绝不调用 `client.load_world()`；沿用 LEAD evaluator 补丁复用当前 world，且服务端 `DefaultEngine.ini` 固定为 Town01。第二个客户端独立 `world.tick()` 也会破坏同步，因此本模块**只**在 evaluator 拥有的 tick 内通过 py_trees 行为更新行人并采样。

### 2. XML 自定义参数透传
确认 `route_parser.py` 中 `other_parameters[elem.tag] = elem.attrib`，因此可用 `<ghost_params .../>` 传参；`type="GhostPedestrian"` 由 `glob srunner/scenarios/*.py` 自动发现，无需改 `route_scenario.py`。

### 3. `DriveDistance` 导入位置
`DriveDistance` 在 `atomic_trigger_conditions`，不在 `atomic_behaviors`。最初导入错误导致场景类导入失败。

### 4. `CarlaDataProvider.get_velocity()` 返回 float
按向量 `ev.x/ev.y` 使用会抛异常并使整个行为树停止（表现为轨迹为空、无 summary）。改为 `float(get_velocity(ego))`。

### 5. 遮挡车生成高度
`vehicle.lincoln.mkz_2020` 在 z=0.3 / 0.5 生成失败，必须 z=1.0；行人则相反，只能在 z=0.0 生成。另外路侧 `x≈398.3–399.0` 存在静态障碍，行人无法在此横向位置生成，只有 `x≤397.8` 或 `x≈399.5` 可行。最终把行人放在 `x=399.48`（lateral 3.2），遮挡车放在 `x=399.08`（lateral 2.8），两者纵向错开 5 m。

### 6. 遮挡车太靠近车道会把 LEAD 逼停
把遮挡车放在 `x=398.28`（lateral 2.0）时，LEAD 在触发点前就把车停死（自车 `y≈232.66`，`brake=1.0`，持续 90+ s），评估超时。退回 lateral 2.8 后 `Occluder Only` 控制组与全部 GP 运行均正常通过。**遮挡车必须留在路肩，不能侵入自车车道。**

### 7. `KeepVelocity` 对行人无效
`KeepVelocity` 对 walker 通过 `apply_control(WalkerControl)` 驱动；`set_simulate_physics(False)` 后行人完全不动（trace 中 `walker_x` 全程恒定）。改为自定义 `DashWalker`，每个 tick 直接 `set_transform`，确定性推进。

### 8. py_trees 重复初始化导致行人「瞬移回起点」
`Sequence` 非记忆模式下 `DashWalker` 会被重新 `initialise()`，行人每约 3.5 s 被拉回起点（trace 中 `walker_x` 在 399.48↔394.28 间反复跳变）。改为用 `GameTime.get_time() - state.release_time` 的**绝对时间**计算位置，重初始化后不再回退。

### 9. `World.cast_ray` 有未文档化的短射程
用 `cast_ray` 从自车相机射向行人，在 25 m 触发距离处（射线长约 23–25 m）始终返回无遮挡；近距离（≤约 20 m）却能命中遮挡车。这是 API 射程限制，不是真实遮挡缺失。改为**显式射线—OBB 采样检测**（60 个采样点，按遮挡车 yaw 反旋到局部坐标判断是否在包围盒内），结果释放前遮挡率 0.75–0.83。

### 10. 传感器在解释器退出时导致 SIGABRT
早先在 `__del__` 里 `stop()` 传感器会触发 "attempting to unsubscribe from stream but sensor wasn't listening"，并在 `Registering the global statistics` 之后 `terminate called without an active exception`（rc=-6）。改为在 `Finalize` 行为中显式 `stop()+destroy()` 相机与碰撞传感器，并把传感器**不**放进 `other_actors`，避免基类二次销毁；之后运行 rc=0。

### 11. 评估超时导致数据丢失
场景未完成时 `Finalize` 不会执行，早先版本只在 `dump()` 里一次性写 trace，超时后只剩空文件。改为每条记录生成后立即追加写入 `trace.jsonl`，并在 `summary.json` 写入 `status: running/final`，任何中断都能保留证据并区分是否 finalize。

## 限制与后续

- 仅 Town01、单一路段、晴天、单一自车起点与单一车速；未做跨地图/跨天气泛化。
- 每个触发距离目前只跑 1 次（seed 0）；如需置信度应增加重复次数。
- 遮挡判据为几何 OBB，另存了 ego 相机帧（`frames/*.png`）作为视觉证据，当前未做图像自动判别。
- 行人在 x=399.48→394.28 的直线路径会穿过路侧静态障碍带，但因直接设置 transform 不受物理约束；视觉上可能有短暂穿模，不影响几何冲突与碰撞检测。
- 若要继续细分临界点，可增加 `gp_20m` / `gp_18m` 等路线（`gen_routes.py` 中的循环即可），其余参数不变。
