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

✅ 鬼探头 v4 场景重建（2026-11-24）：`Town05_Opt` 城市路段、路侧停车遮挡、物理行人、LEAD 前视相机视角录制；几何自检、距离扫描、边界重复、Oracle 对照与 LEAD 感知对齐见 [`docs/GHOST_PEDESTRIAN_V4.md`](docs/GHOST_PEDESTRIAN_V4.md)，代码在 [`ghost_pedestrian_v4/`](ghost_pedestrian_v4/)。

## 接口边界

- **Observation**：CARLA RGB 相机（前视），`sensor_tick=0.0`，每 tick 同帧落盘。
- **Control**：目前是占位策略（直行 / 碰撞后制动），输出 `carla.VehicleControl`。
- **DrivingPolicy / Scenario**：后续接入开源端到端模型与鬼探头场景参数化。
- 仓库只提交代码与配置；CARLA 二进制、conda 环境、PNG/CSV/视频均通过 `.gitignore` 排除。
