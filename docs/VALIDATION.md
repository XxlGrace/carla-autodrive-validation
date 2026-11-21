# CARLA 0.9.16 安装验证记录 (M0–M2)

验证日期：2026-11-21
服务器：AutoDL Ubuntu 22.04 / RTX 4090 24GB / CUDA driver 570.124.04

## 环境证据

| 检查项 | 结果 |
|---|---|
| `nvidia-smi` | NVIDIA GeForce RTX 4090, 24564 MiB |
| `vulkaninfo --summary` | `PHYSICAL_DEVICE_TYPE_DISCRETE_GPU` = NVIDIA GeForce RTX 4090 (api 1.4.303)；另有 CPU `llvmpipe`（正常，忽略） |
| CARLA Server | 0.9.16 (UE4 4.26.2), `-RenderOffScreen -nosound -quality-level=Low -carla-rpc-port=2000` |
| 运行用户 | `carla-runner`（非 root，避免 UE4 `Refusing to run with root privileges`） |
| Python 环境 | `/root/autodl-tmp/envs/carla-py310`, Python 3.10, NumPy 1.26.4 |
| CARLA Python API | 0.9.16，从 `PythonAPI/carla/dist/carla-0.9.16-cp310-*.whl` 安装 |

## 验证结果

### 1. Smoke test (`python -m carla_safety.smoke`)

```
CARLA client version: 0.9.16
CARLA server version: 0.9.16
map: Carla/Maps/Town10HD_Opt
actors: 23
SMOKE OK
```

### 2. 闭环 RGB 测试 (`python -m carla_safety.run`)

运行目录：`/root/autodl-tmp/carla-runs/20261121-194746/`

| 指标 | 结果 |
|---|---|
| steps / 保存帧数 | 200 / 185 |
| 图像规格 | PNG 640×360 RGBA，单帧约 570KB，总计 104MB |
| 像素统计（首/中/末帧） | mean RGB ≈ [162,149,128] / [172,143,126] / [178,149,134]；std ≈ 52–55；唯一色 45k–68k → **真实渲染，非黑图** |
| 车辆 x 位移 | -64.64 → 12.68（约 77m） |
| 速度变化 | 0.09 → 12.15 m/s |
| telemetry.csv | 200 行，含 frame / image_frame / x/y/z / speed / throttle / brake / collision |

结论：真实 RGB 渲染 + 逐帧遥测 + 车辆状态随控制变化，M0–M2 底座打通。

## 已知问题与处理（重要）

1. **运行时 `client.load_world()` 会触发 UE4 SIGSEGV 崩溃**。
   处理：不在客户端切图；地图在服务端启动时决定（默认 `Town10HD_Opt`）。
   `configs/demo.yaml` 中的 `map` 字段仅作记录。若需换图，改 `start_carla.sh`
   的 `-map` 参数并重启服务端，不要在客户端 `load_world()`。

2. **同步模式下 `world.tick()` 已阻塞等待帧完成**，紧接 `world.wait_for_tick()`
   会死锁（等一个永远不会来的 tick）。已移除 `wait_for_tick()`。

3. CARLA 0.9.16 的 Python wheel **没有 `carla.__version__` 属性**。用
   `client.get_server_version()` 获取版本，`getattr(carla, "__version__", "0.9.16")` 兜底。

4. `CarlaUE4.sh` 内 `chmod +x` 对 `carla-runner` 报 `Operation not permitted`，
   但二进制已通过 ACL 赋予 `r-x`，不影响启动，可忽略。

5. 初始质量用 `Low` 完成连通性验证；正式光照/感知实验应改 `High`。

## 复现命令

```bash
bash scripts/start_carla.sh
tail -f /root/autodl-tmp/carla-logs/server.log

source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/carla-py310
python -m carla_safety.smoke
python -m carla_safety.run --config configs/demo.yaml --output-root /root/autodl-tmp/carla-runs

bash scripts/stop_carla.sh
```
