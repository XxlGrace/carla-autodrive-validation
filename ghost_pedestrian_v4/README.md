# GhostPedestrianV4 — 城市路侧停车遮挡的鬼探头

v4 重建了场景本身，保留 v3 的实验框架（LEAD 闭环、Oracle 对照、逐帧遥测、
危险参数扫描）。**v3 的临界距离不能迁移到 v4**：道路、遮挡物、行人运动、
相机与遮挡判定都变了。

结果与分析见 [`../docs/GHOST_PEDESTRIAN_V4.md`](../docs/GHOST_PEDESTRIAN_V4.md)。

> 仓库里不再保留 v0–v3 的鬼探头场景与产物（道路、遮挡物、录制方式都被判定为不合理），
> 它们只存在于 git 历史；本目录与 `docs/GHOST_PEDESTRIAN_V4.md` 是当前唯一有效的版本。

## 场景

`Town05_Opt` 第 19 号路 −1 车道（城市街道，右侧依次是 Parking / Shoulder /
Sidewalk）。由地图推导，不是硬编码偏移：

| 元素 | 规则 | 实测 |
|---|---|---|
| 横穿点 | 路线 `trigger_point` 沿自车车道前进 `crossing_ahead=55 m` | road 19, lane −1, s≈185 |
| 遮挡车 | Nissan Patrol 停进真实 **Parking** 车道，车头停在横穿线前 `occluder_nose=1.2 m` | 4.60×1.93×1.85 m，底面高出路面 **8 mm** |
| 行人起点 | 车头外侧 1.2 m 的可行走带（横向 4.0 m） | 见几何自检：8–40 m 全程可见度 **0.00** |
| 行人运动 | 物理行人 + `carla.WalkerControl`，朝向沿行走方向，1.5 m/s | 9.0 m 横穿，`walker_mode=physics` |
| 遮挡判定 | 行人身体 15 点，从 **LEAD 前视相机**（0.25, 0, 2.25, 384², fov 60）测可见度，含视锥与车身 OBB | 释放前遮挡 0.89，首次可见 ≈1.6 s |

行人起点为什么不放人行道中心：`scripts/verify_geometry_v4.py` 实测，人行道中心
（5.4 m）的行人在 12 m 外已经完全可见，构不成遮挡；紧贴车头外侧（4.0 m）才在
8–40 m 全程不可见。

## 记录
- 每个仿真 tick 一帧（20 Hz），三路：**LEAD 前视相机本体** / 俯视 / 追尾。
- 帧按 `image.frame` 落盘并写 `frames_manifest.jsonl`；编码前按帧号重排、补帧，
  按 20 fps 编码，时间轴与仿真一致。
- `trace.jsonl` 每 tick 一行：自车速度/控制、行人位置、`occlusion`（0 可见 –
  1 全遮挡）、`in_fov`、逐 tick 减速度、E1–E5 所需的时刻。
- 正式实验 `save_frames=0`，完全不创建额外相机（省显存、避免无头渲染器崩）。
- 画质：正式实验与演示都用 `Low`（High 会改变 LEAD 的输入分布，不能混用）。

## 运行

```bash
# 一次跑完：几何自检 → 实验 → 演示 → 分析 → 文档 → 视频
bash scripts/run_all_v4.sh 420
```

或者分步：

```bash
conda activate /root/autodl-tmp/envs/lead
bash /root/autodl-tmp/v4_set_map.sh Town05_Opt        # 启动前定图，评估中不 load_world
python3 scripts/gen_routes_v4.py
bash scripts/install_v4.sh

# LEAD 侧感知钩子（pred 运行需要；本机已应用，重装 LEAD 后要重打）
patch -p0 -d /root/autodl-tmp/third_party/lead/src/lead/evaluation/agents/transfuser \
      < patches/gp_pred_hook.patch

# 几何自检（只连 CARLA，不跑模型）
bash /root/autodl-tmp/v4_start_carla.sh 2000 Low
python3 scripts/verify_geometry_v4.py > results/geometry_check.txt

# 批量：路线列表每行 "<file.xml> [pred]"，pred 打开 LEAD 感知钩子
bash scripts/run_v4.sh /tmp/v4_sweep.txt 420
python3 scripts/analyze_v4.py
python3 scripts/gen_v4_doc.py
python3 scripts/make_demos_v4.py
```

`run_v4.sh` 每条路线重启一次 CARLA。部分运行返回码 250：无头构建在**写完
summary 之后**退出时 abort；分析以 `summary.status == "final"` 为成功判据。

## 目录

```
scenarios/ghost_pedestrian_v4.py   scenario runner 场景
patches/gp_pred_hook.patch         LEAD 推理钩子（GP_PRED_LOG），pred 运行必需
scripts/gen_routes_v4.py           生成路线 XML
scripts/install_v4.sh              装进 LEAD 的 scenario_runner
scripts/run_v4.sh                  批量执行（逐条重启 CARLA）
scripts/verify_geometry_v4.py      几何/贴地/可见性自检
scripts/analyze_v4.py              扫描、重复、Oracle、E1–E5、感知对齐
scripts/gen_v4_doc.py              生成本文件的数据部分
scripts/make_demos_v4.py           四格演示视频（含逐帧 HUD）
routes/                            路线 XML
results/                           CSV/JSON 结果与几何自检输出
demos/                             演示视频
```

## 已知限制

- 行人步态是 CARLA 程序化动画，不是动作捕捉的真实步态。
- 遮挡率是 15 点采样，不是像素级/语义级遮挡。
- LEAD 传感器帧与推理帧相差 1–2 tick，未单独补偿。
- 只验证了一个路段；跨路段泛化（v4.3）与危险参数自动生成尚未做。
