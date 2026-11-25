# GhostPedestrianV4 × VAD（Bench2DriveZoo VAD-Base）

在同一段鬼探头场景（`Town05_Opt`、路侧停放的 Nissan Patrol、行人 1.5 m/s、同一 trigger
点、同一天气、20 Hz 同步）上把被测模型换成 **VAD-Base**，专门回答一个问题：
**VAD 到底对什么因素踩刹车？**

- LEAD 的原始结果：[`../docs/GHOST_PEDESTRIAN_V4.md`](../docs/GHOST_PEDESTRIAN_V4.md)（CARLA 0.9.16）
- 本实验的 CARLA 是 **0.9.13**（服务器现有版本）→ 跨版本探索性对比
- 明细：[`results/comparison.md`](results/comparison.md) · 因子对照：[`results/factor_controls.md`](results/factor_controls.md) ·
  控制器对照：[`results/controller_variants.md`](results/controller_variants.md) · 逐次指标：[`results/window_metrics.csv`](results/window_metrics.csv)

## 结论先说

1. **空路上不刹**：进入 20 m 后一路 6.9 m/s，直到路口（距横穿线 1.4 m）才点一脚。
2. **只放那辆停着的 SUV，VAD 就会停**：制动起点 0.45–0.80 s、期望速度降到 1.8–2.2、
   车速到 0.01 m/s。
3. **遮挡+行人时的早刹（0.70–0.75 s）与“仅遮挡车”几乎相同** → 触发源是 SUV，不是被挡住的行人。
4. **行人只在可见时才额外起作用**：去掉遮挡后制动提前到 0.20–0.25 s。
5. 真鬼探头下 VAD 为 SUV 停下后，行人在 1.6 s 才露出，它**随后又加速**，以 5.2–7.6 m/s
   从行人旁 1.5–1.8 m 处开过去（不是“停住等行人”）。
6. **控制器不是瓶颈**：加制动保持 0.3 s / 把释放阈值改成 0.95 的滞回，脉冲从 3–5 段降到 2–3 段、
   最长连续制动从 0.2–0.3 s 拉到 0.35–0.55 s，但通过车速与最小距离没有实质变化
   —— 因为“停车后又起步”是**规划器**的决定（`desired_speed` 从 ~1.5 回升到 4–5）。

## 四组因子对照（20 m，各 3 次）

| 条件 | 制动起点 τ | 3 s 占空 | 最低车速 | 期望速度最低 | 通过行人车速 | 最近距离 |
|---|---|---|---|---|---|---|
| 空道路 | 无（τ≈3 才 1 tick） | 0.02–0.03 | 6.87–6.94 | 5.50–6.64 | — | — |
| 仅遮挡车 | 0.45–0.80 | 0.08–0.13 | 0.01–0.02 | 1.82–2.16 | — | — |
| 仅行人 | 0.20–0.25 | 0.20–0.27 | 0.00–0.07 | 2.12–2.55 | 6.90–7.83 | 1.73–2.15 |
| 遮挡车+行人 | 0.70–0.75 | 0.15–0.20 | 0.02–1.03 | 1.46–2.54 | 5.21–7.62 | 1.46–1.79 |

LEAD 在所有运行里的制动起点都 ≥ 行人首次可见时刻（1.85–3.05 s ≥ 1.55–1.70 s），即它只对行人反应；
VAD 的 0.45–0.80 s 远早于可见时刻，触发源不同。这就是“LEAD 撞、VAD 不撞”的直接原因。

## LEAD vs VAD（距离档）

| 距离 | LEAD 碰撞 | VAD 碰撞 | LEAD 最近距离 | VAD 最近距离 |
|---|---|---|---|---|
| 20 m（速度匹配 7.5–8.0 m/s） | **2 / 3** | **0 / 3** | 1.09–1.13 m | 1.46–1.79 m |
| 24 m（VAD 5.2–5.7 m/s ⚠） | 0 / 3 | 0 / 3 | 2.61–4.01 m | 1.67–1.75 m |
| 26 m（VAD 4.6–4.9 m/s ⚠） | **2 / 3** | 0 / 3 | 2.20–2.50 m | 2.11–2.24 m |

⚠ 24/26 m 的速度不匹配：VAD 车速在 ~7.6→6.5→4.7→8.0 之间振荡，按距离触发时落在波谷。

## 统一时间窗（第一批修正）

之前的表格把“释放后全窗口最低车速”当成“制动效果”，把 VAD 在路线末尾的停车也算进去了。
现在统一为：

- 锚点 τ=0 = 自车第一次进入“距横穿线 trigger_distance”的那一 tick（有/无行人都适用）；
- 场景流与模型流按**仿真帧号**对齐，实测两套时钟固定相差 **0.05 s（1 tick）**；
- 统计窗口 τ∈[−2, +5] s，分别给出“3 s 内最低车速 / 通过行人时车速 / 最近距离”。

按这个口径，VAD 在鬼探头里并不是“停住”，而是“为 SUV 停一下 → 行人露出 → 再加速通过”。
参数化实现：`scripts/analyze_window.py`。

## 环境与依赖

| 项 | 值 |
|---|---|
| CARLA | 0.9.13（`Town05_Opt` 由 `load_world` 切换） |
| Python | 3.8（conda env） |
| CUDA / torch | CUDA 11.8 toolkit（conda `nvidia/label/cuda-11.8.0`）+ `torch==2.0.1+cu118` |
| 代码 | Bench2Drive `0.0.4` + Bench2DriveZoo `uniad/vad-0.0.4` |
| 权重 | `vad_b2d_base.pth` 668 MB + `resnet50-19c8e357.pth`，经 hf-mirror |
| 场景 | GitHub 上的 v4 场景（未改几何） |
| 相机 | 6×1600×900 RGB（fov 70）+ IMU/GNSS/速度；不含 bev 相机 |

### 为 0.9.13 打的兼容补丁（`patches/`）

| 补丁 | 原因 |
|---|---|
| `patch_lights.py` | `LightManager` 没有 `set_day_night_cycle` |
| `patch_settings.py` | `WorldSettings` 没有 `spectator_as_ego` |
| `patch_eval.py` | 评估器复用外部 CARLA（`VAD_EXTERNAL_CARLA=1`） |
| `patch_agent2.py` | `IS_BENCH2DRIVE=False` 分支引用未定义的 `now`；bev 相机 z=50 m 触发外参上限 |
| `patch_agent3.py` | `RoadOption` 枚举被当 int 用 |
| `patch_tm.py` | 背景交通用到 0.9.14+ 的 13 个 `TrafficManager` 方法 |
| `patch_hook2.py` | 逐帧 JSONL 钩子放进 `run_step()`（`VAD_TRACE_JSONL`） |
| `patch_brake.py` | **控制器对照**：`VAD_BRAKE_HOLD_S` / `VAD_BRAKE_HYST`（默认关闭） |

## 运行清单（`results/`）

| tag 前缀 | 条件 | 次数 |
|---|---|---|
| `d20m_r1..r3` / `d24m_r1..r3` / `d26m_r1..r3` | 遮挡车 + 行人 | 各 3 |
| `noocc_d20m_r1..r3` | 仅行人（无遮挡车） | 3 |
| `occonly_d20m*` | 仅遮挡车（无行人） | 3 |
| `empty_d20m_r1..r3` | 空道路 | 3 |
| `hold_d20m_r1..r3` | 遮挡+行人，制动保持 0.3 s | 3 |
| `hyst_d20m_r1..r3` | 遮挡+行人，滞回 0.95 | 3 |
| `long_d26m_r1` | 加长助跑探针（失败：VAD 提前停车，场景未释放） | 1 |

每次运行有 `summary_<tag>.json`（场景侧）、`vad_trace_<tag>.jsonl`（模型侧逐帧）、
`scenario_trace_<tag>.jsonl`（距离/遮挡/横向）、`eval_<tag>.log`。

## 复现

```bash
conda create -y -p envs/vad-b2d python=3.8
conda install -y -p envs/vad-b2d -c https://conda.anaconda.org/nvidia/label/cuda-11.8.0 \
      --override-channels cuda-toolkit=11.8
envs/vad-b2d/bin/pip install torch==2.0.1+cu118 torchvision==0.15.2+cu118 torchaudio==2.0.2+cu118 \
      --index-url https://download.pytorch.org/whl/cu118
git clone --depth 1 -b 0.0.4 https://github.com/Thinklab-SJTU/Bench2Drive.git
git clone --depth 1 -b uniad/vad-0.0.4 https://github.com/Thinklab-SJTU/Bench2DriveZoo.git
(cd Bench2DriveZoo && pip install -v -e . --no-deps)     # 需要 setuptools<71
curl -L -o Bench2Drive/ckpts/vad_b2d_base.pth \
      https://hf-mirror.com/rethinklab/Bench2DriveZoo/resolve/main/vad_b2d_base.pth
pip install ~/CARLA_0.9.13/carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
# 打 patches/、放场景、起 CARLA、跑 routes/ 下的 XML
# 控制器变体：VAD_BRAKE_HOLD_S=0.3 或 VAD_BRAKE_HYST=0.95
```

## 局限

1. **跨 CARLA 版本**（0.9.13 vs 0.9.16）——0.9.16 官方包只带 cp310+ 的 Python 客户端，VAD 栈是
   py3.8 专用，无法直接对齐。
2. **24/26 m 速度不匹配**（VAD 4.6–5.7 vs LEAD 7.2–7.6），只有 20 m 严格可比。
3. 每档 3 次，只能做描述性统计。
4. 保持/滞回只试了 0.3 s / 0.95 两个点。
