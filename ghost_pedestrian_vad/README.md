# GhostPedestrianV4 × VAD（Bench2DriveZoo VAD-Base）—— 首次闭环对照

在同一段鬼探头场景（`Town05_Opt`、路侧停放的 Nissan Patrol、行人 1.5 m/s、同一个
trigger 点、同一套天气/物理）上，把被测模型从 LEAD 换成 **VAD-Base**（Bench2DriveZoo
`uniad/vad-0.0.4` + `vad_b2d_base.pth`），跑通闭环，并记录与 LEAD 同口径的逐帧数据。

- LEAD 的原始结果见 [`../docs/GHOST_PEDESTRIAN_V4.md`](../docs/GHOST_PEDESTRIAN_V4.md)（CARLA 0.9.16）。
- 本目录的实验跑在一台实验室服务器上，**只有 CARLA 0.9.13**，所以是**跨版本探索性对比**，
  不是严格配对实验（见文末“局限”）。
- 结果表：[`results/comparison.md`](results/comparison.md)。

## 20 m 首轮结果（与 LEAD 同档对比）

| | LEAD（0.9.16） | VAD（0.9.13） |
|---|---|---|
| 释放时车速 | 7.51–7.78 m/s | **7.84–8.11 m/s** |
| 行人首次可见（释放后） | 1.55–1.60 s | 1.65 s |
| 持续制动起点 | 2.75–2.80 s | **0.70–0.75 s** |
| 制动后最低车速 | 7.3 m/s（没降下来） | **≈0（完全停住）** |
| 结果（3 次） | **碰撞 2/3** | **0/2 碰撞，最近 1.71–1.75 m** |
| 路线完成度 | 100% | 100%（`MinSpeedTest` 99.8% 不达标） |

**关键机制差异**：VAD 在释放后 0.7 s 就踩到底、1 s 内把车从 8.1 m/s 刹到 0 —— 这发生在
**行人首次可见（1.65 s）之前**。所以它不是“看见了鬼探头才刹”，而是对整段街道/路侧停车
本来就采取了很保守的低速-停车策略；LEAD 则一路保持 7+ m/s，直到 2.8 s 才开始制动。
两者都出现**脉冲式制动**（VAD 20 m 档 86–96 个制动 tick、30 段脉冲；LEAD 24 tick、6 段），
但 VAD 的脉冲能累计到完全停车，LEAD 的不能。

## 24 / 26 m（单次，未做速度匹配）

| 距离 | 释放车速 | 持续制动起点 | 最长连续制动 | 最近距离 | 结果 |
|---|---|---|---|---|---|
| 24 m | 5.22 m/s | 1.40 s | 2.15 s | 1.84 m | 无碰撞 |
| 26 m | 4.61 m/s | 1.65 s | 2.15 s | 2.27 m | 无碰撞 |

⚠️ 这两档 VAD 是**低速接近**（5.2 / 4.6 m/s，LEAD 同档约 7.2–7.6），难度不同，
不能直接与 LEAD 的 24/26 m 结果并列。要做严格对比，需要先让 VAD 以 7.5–8 m/s 接近
（例如加长助跑或换一个更长的路线起点）。

## 环境与依赖（可复现）

| 项 | 值 |
|---|---|
| CARLA | 0.9.13（服务器现有安装，`Town05_Opt` 由 `load_world` 切换） |
| Python | 3.8（conda env） |
| CUDA / torch | CUDA 11.8 toolkit（conda `nvidia/label/cuda-11.8.0`）+ `torch==2.0.1+cu118` |
| 代码 | Bench2Drive `0.0.4` + Bench2DriveZoo `uniad/vad-0.0.4`（clone 到 `Work/XXL`） |
| 权重 | `ckpts/vad_b2d_base.pth`（668 MB）+ `resnet50-19c8e357.pth`（98 MB），经 hf-mirror |
| 场景 | `scenario_runner/srunner/scenarios/ghost_pedestrian_v4.py`（从 GitHub 下载的 v4 场景，未改） |
| 相机 | 6×1600×900 RGB（fov 70）+ IMU/GNSS/速度；**不含** bev 相机 |
| 渲染 | 无头 `-RenderOffScreen -quality-level=Low`；实车 23 s 仿真约 192 s 墙钟（≈0.12×） |

### 为 0.9.13 打的 5 处兼容补丁（详见 `patches/`）

| 补丁 | 原因 |
|---|---|
| `patch_lights.py` | 0.9.13 的 `LightManager` 没有 `set_day_night_cycle` |
| `patch_settings.py` | 0.9.13 的 `WorldSettings` 没有 `spectator_as_ego` 参数 |
| `patch_eval.py` | 让评估器复用外部启动的 CARLA（`VAD_EXTERNAL_CARLA=1`），不自己再拉起一个 |
| `patch_agent2.py` | `IS_BENCH2DRIVE=False` 分支引用了未定义的 `now`；bev 相机 z=50 m 触发“3 m 外参上限”报错 → 改为可选 |
| `patch_agent3.py` | `RoadOption` 是枚举，agent 按 int 处理会抛 `TypeError` |
| `patch_tm.py` | 背景交通用到 0.9.14+ 的 `TrafficManager.vehicle_lane_offset` 等 13 个方法 |
| `patch_hook2.py` | 把 VAD 的逐帧 JSONL 日志钩子放进 `run_step()`（见下） |

### 逐帧日志

在 `vad_b2d_agent.py` 的 `run_step()` 末尾加了一个环境变量开关（`VAD_TRACE_JSONL`），
每个 tick 写一行：`frame / step / t / speed / desired_speed / steer / throttle / brake / plan_end`，
口径与 LEAD 的 `trace.jsonl` 对齐。

## 复现步骤

```bash
# 1) 环境（py3.8 + cuda11.8 + torch2.0.1）
conda create -y -p envs/vad-b2d python=3.8
conda install -y -p envs/vad-b2d -c https://conda.anaconda.org/nvidia/label/cuda-11.8.0 \
      --override-channels cuda-toolkit=11.8
envs/vad-b2d/bin/pip install torch==2.0.1+cu118 torchvision==0.15.2+cu118 torchaudio==2.0.2+cu118 \
      --index-url https://download.pytorch.org/whl/cu118
# 2) 代码 + 权重
git clone --depth 1 -b 0.0.4 https://github.com/Thinklab-SJTU/Bench2Drive.git
git clone --depth 1 -b uniad/vad-0.0.4 https://github.com/Thinklab-SJTU/Bench2DriveZoo.git
(cd Bench2DriveZoo && pip install -v -e . --no-deps)     # 需要 setuptools<71，否则 pkg_resources.packaging 报错
curl -L -o Bench2Drive/ckpts/vad_b2d_base.pth \
      https://hf-mirror.com/rethinklab/Bench2DriveZoo/resolve/main/vad_b2d_base.pth
# 3) CARLA 0.9.13 客户端（用安装包里自带的 cp38 wheel）
pip install ~/CARLA_0.9.13/carla-0.9.13-cp38-cp38-manylinux_2_27_x86_64.whl
# 4) 打补丁 + 放场景 + 起 CARLA + 跑
```

## 局限

1. **跨 CARLA 版本**：VAD 在 0.9.13，LEAD 在 0.9.16；地图虽同为 `Town05_Opt`，但版本间
   路网/物理有差异，结论只能算探索性。
2. **速度不匹配**：24/26 m 两档 VAD 以 4.6–5.2 m/s 接近，不能与 LEAD 的 7+ m/s 并列。
3. **样本量**：20 m 只跑了 2 次，24/26 m 各 1 次，还不足以谈碰撞率。
4. **“没撞”的归因**：VAD 的停车发生在行人可见之前，更像是保守驾驶策略，而不是
   “识别到鬼探头”。要分离二者，需要一组「无行人/无遮挡」对照看它是否照样在那个位置停车。
5. 6 路 1600×900 渲染很重，23 s 仿真要 192 s 墙钟（0.12×），批量实验前要评估时间成本。
