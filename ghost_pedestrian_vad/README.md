# GhostPedestrianV4 × VAD（Bench2DriveZoo VAD-Base）

在同一段鬼探头场景（`Town05_Opt`、路侧停放的 Nissan Patrol、行人 1.5 m/s、同一 trigger
点、同一天气、20 Hz 同步）上，把被测模型从 LEAD 换成 **VAD-Base**（Bench2DriveZoo
`uniad/vad-0.0.4` + `vad_b2d_base.pth`），跑完 20 / 24 / 26 m 各 3 次，另加无遮挡
对照 3 次与仅遮挡车 1 次。

- LEAD 的原始结果：[`../docs/GHOST_PEDESTRIAN_V4.md`](../docs/GHOST_PEDESTRIAN_V4.md)（CARLA 0.9.16）
- 本实验跑在实验室服务器上，只有 **CARLA 0.9.13** → **跨版本探索性对比**，不是严格配对实验
- 明细表：[`results/comparison.md`](results/comparison.md)、[`results/comparison.csv`](results/comparison.csv)、逐次指标 [`results/vad_runs.csv`](results/vad_runs.csv)

## 结果

| 距离 | LEAD 碰撞 | VAD 碰撞 | LEAD 最近距离 | VAD 最近距离 |
|---|---|---|---|---|
| 20 m（速度匹配 7.5–8.0 m/s） | **2 / 3** | **0 / 3** | 1.09–1.13 m | 1.46–1.79 m |
| 24 m（VAD 仅 5.2–5.7 m/s ⚠️） | 0 / 3 | **0 / 3** | 2.61–4.01 m | 1.67–1.75 m |
| 26 m（VAD 仅 4.6–4.9 m/s ⚠️） | **2 / 3** | **0 / 3** | 2.20–2.50 m | 2.11–2.24 m |
| 合计 | **4 / 9** | **0 / 12**（含无遮挡 3 次） | | |

- VAD 每次都把车**刹到 ≈0 m/s**（12/12），LEAD 在碰撞时仍保持 6.7–7.4 m/s。
- 无遮挡对照（有行人、无 SUV）：VAD 仍在 0.25–0.30 s 就制动，比有遮挡时（0.75–1.65 s）
  更早 → 它的制动更像“接近路口就停车”的保守策略，而不是“识别到鬼探头”。
- 仅遮挡车（无行人）一次：无碰撞，作为场景基线。

## 重要解读与局限

1. **“没撞”不等于“识别得好”**：VAD 的制动发生在行人首次可见（1.6 s）之前，且无遮挡时
   更早。要分离二者，需要“行人在远处静止/不存在”的对照，看它是否照样在那个位置停车。
2. **24/26 m 速度不匹配**：VAD 车速在 ~7.6→6.5→4.7→8.0 m/s 之间振荡，按距离触发时正好
   落在波谷；LEAD 同档是 7.2–7.6 m/s。所以只有 20 m 是严格可比的。
3. **加长助跑会卡住**：给 VAD 多 40 m 助跑后，它在到达 trigger 前就停车不动（`desired_speed≈0`，
   t=63 s 仍未释放行人）。VAD 在这个路段会主动选择停住。
4. **跨版本**：CARLA 0.9.13 vs 0.9.16；VAD 用 6×1600×900 相机、LEAD 用 6×384²，属模型固有差异。
5. 6 路 1600×900 渲染很重：23 s 仿真约 190 s 墙钟（0.12×）。

## 环境与依赖

| 项 | 值 |
|---|---|
| CARLA | 0.9.13（服务器现有；`Town05_Opt` 由 `load_world` 切换） |
| Python | 3.8（conda env） |
| CUDA / torch | CUDA 11.8 toolkit（conda `nvidia/label/cuda-11.8.0`）+ `torch==2.0.1+cu118` |
| 代码 | Bench2Drive `0.0.4` + Bench2DriveZoo `uniad/vad-0.0.4` |
| 权重 | `vad_b2d_base.pth`（668 MB）+ `resnet50-19c8e357.pth`，经 hf-mirror |
| 场景 | `scenario_runner/srunner/scenarios/ghost_pedestrian_v4.py`（GitHub 上的 v4 场景，未改） |
| 相机 | 6×1600×900 RGB（fov 70）+ IMU/GNSS/速度；不含 bev 相机 |

### 为 0.9.13 打的兼容补丁（`patches/`）

| 补丁 | 原因 |
|---|---|
| `patch_lights.py` | 0.9.13 的 `LightManager` 没有 `set_day_night_cycle` |
| `patch_settings.py` | 0.9.13 的 `WorldSettings` 没有 `spectator_as_ego` 参数 |
| `patch_eval.py` | 评估器复用外部 CARLA（`VAD_EXTERNAL_CARLA=1`），不自己再拉起一个 |
| `patch_agent2.py` | `IS_BENCH2DRIVE=False` 分支引用未定义的 `now`；bev 相机 z=50 m 触发 3 m 外参上限 |
| `patch_agent3.py` | `RoadOption` 是枚举，agent 按 int 处理会抛 `TypeError` |
| `patch_tm.py` | 背景交通用到 0.9.14+ 的 13 个 `TrafficManager` 方法 |
| `patch_hook2.py` | 把逐帧 JSONL 钩子放进 `run_step()`（`VAD_TRACE_JSONL`） |

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
# 打补丁、放场景、起 CARLA、跑 routes/ 下的 XML
```

## 运行清单

`results/` 里每次运行都有：`summary_<tag>.json`（场景侧：释放距离/车速、遮挡率、最近距离、
是否碰撞）、`vad_trace_<tag>.jsonl`（模型侧逐帧 speed/desired_speed/brake/throttle）、
`scenario_trace_<tag>.jsonl`（距离/横向位置/遮挡）、`eval_<tag>.log`（评估器日志）。

| tag | 距离 | 说明 |
|---|---|---|
| `d20m_r1..r3` / `d24m_r1..r3` / `d26m_r1..r3` | 20/24/26 | 主实验，各 3 次 |
| `noocc_d20m_r1..r3` | 20 | 无遮挡对照（有行人） |
| `occonly_d20m` | 20 | 仅遮挡车（无行人） |
| `long_d26m_r1` | 26 | 加长助跑探针（失败：VAD 提前停车） |
