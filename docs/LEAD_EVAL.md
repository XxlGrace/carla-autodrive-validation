# LEAD 预训练模型闭环测试（AutoDL）

## 组件与路径

| 项目 | 值 |
|---|---|
| 代码 | `/root/autodl-tmp/third_party/lead`（kesai-labs/lead `main`，HEAD `3764f79`）|
| 环境 | `/root/autodl-tmp/envs/lead`（Python 3.10 + PyTorch 2.8.0+cu128）|
| 权重 | `third_party/lead/checkpoints/transfuser/`（推荐 `resnet34_v1.5.0/seed0`，264MB，Bench2Drive DS 93.6）|
| 评估路线 | `src/lead/routes/benchmark_routes/longest6/00.xml`（Town01）|
| 日志 | `/root/autodl-tmp/lead_full_eval.log` |

## 启动前必须满足的四个条件

1. **服务端地图 = 路线 town**。客户端 `client.load_world()` 在本机无头 Vulkan 构建上会触发 UE4 `Signal 11`（SIGSEGV），使服务端崩溃。做法：把 `/root/autodl-tmp/CARLA_0.9.16/CarlaUE4/Config/DefaultEngine.ini` 中 `GameDefaultMap/EditorStartupMap/ServerDefaultMap` 设为路线对应地图（本例 Town01），再重启服务端。
2. **打过 evaluator 补丁**。`3rd_party/.../leaderboard_evaluator.py::_load_and_wait_for_world` 中若当前地图已等于目标 town，直接复用 `get_world()`，跳过 `load_world()`。备份在 `leaderboard_evaluator.py.orig_autodl`。
3. **`NUMBA_NUM_THREADS=24 OMP_NUM_THREADS=24`**（关键性能修复）。机器 cgroup 只给 25 CPU，但 numba/OpenMP 默认起 224 线程，严重超订，导致 RANSAC 地面分割耗时约 3.5s/步；限制到 24 后降到约 0.035s/步（约 100×）。
4. **离线权重已预缓存**。模型构建时 `timm.create_model("resnet34", pretrained=True)` 会下载 ImageNet 权重；已缓存 `timm/resnet34.a1_in1k/model.safetensors` 到 `~/.cache/huggingface`，配合 `HF_HUB_OFFLINE=1` 使用。另需系统 `ffmpeg`（本例 apt 安装 4.4.2）用于视频录制。

`scripts/start_carla.sh` 已固定 ICD 为 `/etc/vulkan/icd.d/nvidia_icd.json` 并加 `-graphicsadapter=0`，确保渲染落在 RTX 4090。

## 跑一条路线

```bash
# 0) 若换路线 town，先改 DefaultEngine.ini 三个 *DefaultMap 并重启服务端
bash /root/autodl-tmp/restart_carla_clean.sh

# 1) 评估（~/autodl proxy 供 huggingface/wandb 兜底；本地 CARLA 走 no_proxy）
source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/lead
cd /root/autodl-tmp/third_party/lead
export PYTHONUNBUFFERED=1 HF_HUB_OFFLINE=1 WANDB_MODE=offline
export NUMBA_NUM_THREADS=24 OMP_NUM_THREADS=24
export http_proxy=http://127.0.0.1:1080 https_proxy=http://127.0.0.1:1080 no_proxy=127.0.0.1,localhost,::1
python -m lead --checkpoint checkpoints/transfuser \
  --routes src/lead/routes/benchmark_routes/longest6/00.xml \
  --port 2000 --timeout 300
```

输出目录：`outputs/local_evaluation/00/`，含 `checkpoint_endpoint.json`、逐场景结果表。

## 已知问题

- `client.load_world()` → SIGSEGV（见上，补丁 + 服务端定图规避）。
- “Problem reading checkpoint. Found no data” 是 evaluator 的 resume 端点文件不存在，首次运行正常，可忽略。
- 场景 `SignalizedJunctionLeftTurn_*`、`OppositeVehicleRunningRedLight_*` 在本地图会因找不到对向车道被跳过，属地图/路线组合问题。

## 实测结果（2026-11-21，完整路线）

`longest6/00.xml`（Town01，11 个场景）完整跑通，日志 `/root/autodl-tmp/lead_full_eval.log`：

| 指标 | 结果 |
|---|---|
| Route Completion | **100%** |
| Collision | 0 |
| Running Red Light | 0 |
| Running Stop | 0 |
| Outside Route Lanes | 0% |
| 其余 6 项 | 全部 SUCCESS |
| **Driving Score** | **100**（100% × 无惩罚）|
| 用时 | System 766.9s / Game 236.7s，Ratio 0.309× |
| 退出码 | 0 |

结论：推荐的 `resnet34_v1.5.0/seed0` 权重在本机无头环境下闭环可用，单条路线满分。
