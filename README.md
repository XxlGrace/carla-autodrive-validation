# CARLA 自动驾驶安全验证底座

AutoDL × CARLA 0.9.16 无头（headless）闭环实验底座。

## 目录与版本

| 项目 | 路径 / 版本 |
|---|---|
| CARLA Server | `/root/autodl-tmp/CARLA_0.9.16` (0.9.16, Vulkan, `-RenderOffScreen`) |
| Python 环境 | `/root/autodl-tmp/envs/carla-py310` (Python 3.10, NumPy 1.26.4) |
| 本仓库 | `/root/autodl-tmp/carla-autodrive-validation` |
| 实验产物 | `/root/autodl-tmp/carla-runs/` (不提交) |
| CARLA 日志 | `/root/autodl-tmp/carla-logs/` |

## 快速开始

```bash
# 1) 启动 CARLA（非 root，Vulkan 离屏）
bash scripts/start_carla.sh
tail -f /root/autodl-tmp/carla-logs/server.log

# 2) 激活环境
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate /root/autodl-tmp/envs/carla-py310

# 3) 连通性检查
python -m carla_safety.smoke

# 4) 跑一次闭环（真实 RGB 图像 + 逐帧遥测）
python -m carla_safety.run --config configs/demo.yaml --output-root /root/autodl-tmp/carla-runs

# 5) 停用
bash scripts/stop_carla.sh
```

## 验证状态

✅ M0–M2 底座已跑通（2026-11-21）：CARLA 0.9.16 服务端 + 非 root 离屏渲染 + 真实 RGB 闭环 + 逐帧遥测。详细证据与已知问题见 [`docs/VALIDATION.md`](docs/VALIDATION.md)。

✅ 端到端基线 LEAD 预训练模型（TransFuser / resnet34_v1.5.0）已在本机完成闭环评估环境搭建与路线验证；启动前置条件、性能修复（numba 线程数与 cgroup 配额对齐）及踩坑记录见 [`docs/LEAD_EVAL.md`](docs/LEAD_EVAL.md)。

✅ 鬼探头 v4 场景重建（2026-11-24）：`Town05_Opt` 城市路段、路侧停车遮挡、物理行人、LEAD 前视相机视角录制；几何自检、距离扫描、边界重复、Oracle 对照与 LEAD 感知对齐见 [`docs/GHOST_PEDESTRIAN_V4.md`](docs/GHOST_PEDESTRIAN_V4.md)，代码在 [`ghost_pedestrian_v4/`](ghost_pedestrian_v4/)。旧的 v0–v3 场景与产物已移除（只存于 git 历史）。

🧪 VAD 对照（跨版本探索）：同一鬼探头场景下把被测模型换成 VAD-Base，20/24/26 m 各 3 次共 9 次均未碰撞（LEAD 同档 4/9 碰撞），另有 3 次无遮挡对照；但 VAD 的制动早于行人可见、且 24/26 m 接近速度偏低，不能解读为“识别更好”。见 [`ghost_pedestrian_vad/`](ghost_pedestrian_vad/)。

🎬 v4 演示视频：[碰撞（20 m，被遮挡后撞上）](ghost_pedestrian_v4/demos/demo_collision.mp4) · [安全（40 m）](ghost_pedestrian_v4/demos/demo_safe.mp4) · [Oracle 首次可见即刹](ghost_pedestrian_v4/demos/demo_oracle.mp4) · [仅遮挡车基线](ghost_pedestrian_v4/demos/demo_occluder_only.mp4) · [四格对照](ghost_pedestrian_v4/demos/demo_comparison.mp4)

## 接口边界

- **Observation**：CARLA RGB 相机（前视），`sensor_tick=0.0`，每 tick 同帧落盘。
- **Control**：目前是占位策略（直行 / 碰撞后制动），输出 `carla.VehicleControl`。
- **DrivingPolicy / Scenario**：LEAD（TransFuser）端到端模型已接入；鬼探头场景见 `ghost_pedestrian_v4/`（参数化触发距离/行人速度/遮挡位置）。
- 仓库只提交代码与配置；CARLA 二进制、conda 环境、PNG/CSV/视频均通过 `.gitignore` 排除。
