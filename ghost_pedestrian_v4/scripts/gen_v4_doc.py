#!/usr/bin/env python3
"""Generate docs/GHOST_PEDESTRIAN_V4.md from the v4 results."""
import csv
import json
import os

REPO = "/root/autodl-tmp/carla-autodrive-validation"
V4 = os.path.join(REPO, "ghost_pedestrian_v4")
RES = os.path.join(V4, "results")
OUT = os.path.join(REPO, "docs", "GHOST_PEDESTRIAN_V4.md")


def rows(name):
    p = os.path.join(RES, name)
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def load(name):
    p = os.path.join(RES, name)
    return json.load(open(p)) if os.path.exists(p) else {}


def text(name):
    p = os.path.join(RES, name)
    return open(p).read().strip() if os.path.exists(p) else ""


def fmt(x, nd=2, dash="—"):
    if x in (None, "", "None", "nan"):
        return dash
    try:
        return ("%." + str(nd) + "f") % float(x)
    except Exception:
        return str(x)


def yes(v):
    return "**是**" if str(v) == "True" else "否"


sweep = rows("v4_sweep.csv")
controls = rows("v4_controls.csv")
repeats = rows("v4_repeats.csv")
events = load("v4_events.json")
percept = rows("v4_perception.csv")
physics = rows("v4_physics.csv")
frames = rows("v4_frames.csv")
geom = text("geometry_check.txt")

L = []
A = L.append

A("# 鬼探头 v4 —— 城市路侧停车遮挡场景（Town05_Opt）")
A("")
A("v4 不使用 v3 的数值：道路、遮挡物、行人运动、相机位置、遮挡判定方式全部"
  "改变，v3 的临界距离不能迁移到 v4。v4 只保留实验框架（LEAD 闭环、Oracle "
  "对照、逐帧遥测、危险参数扫描）。")
A("")

# ---------------------------------------------------------------- conclusion
coll = [r for r in sweep if r["collision"] == "True"]
safe = [r for r in sweep if r["collision"] != "True"]
A("## 结论摘要")
A("")
if coll:
    A("- 单次距离扫描：**%s m 碰撞**，其余 %d 个距离点安全。" % (
        "、".join("%g" % float(r["trigger_m"]) for r in coll), len(safe)))
    ph0 = {r["run"]: r for r in physics}.get(coll[0]["run"])
    A("- 被遮挡期间（E1）行人身体被遮挡 %s，且完整落在 LEAD 前视相机视场内。"
      % fmt(coll[0]["occ_before"]))
    if ph0 and ph0.get("margin_m") not in (None, ""):
        A("- 首次可见（E3）时自车距行人 %s m、速度 %s m/s；按碰撞前实测的最大"
          "减速度 %s m/s²，刹停只需要 %s m，**空间余量 %s m**。"
          % (fmt(ph0["dist_at_visible_m"]), fmt(ph0["v_at_visible_mps"]),
             fmt(ph0["max_decel_pre_crash"]), fmt(ph0["required_stop_m"]),
             fmt(ph0["margin_m"])))
for r in repeats:
    A("- **%s m 重复 %s 次：碰撞 %s 次（%.0f%%）**，最小距离 %s–%s m。" % (
        fmt(r["trigger_m"], 0), r["runs"], r["collisions"],
        float(r["collision_rate"]) * 100, fmt(r["min_dist_min"], 3),
        fmt(r["min_dist_max"], 3)))
A("")
A("所以 v4 的结论不是「临界距离 = X m」，而是：**边界附近的碰撞是一个概率带**"
  "（20 m 与 26 m 各 2/3 碰撞，24 m 0/3），单次碰撞/安全不能当确定的边界；"
  "可以把结果讲清楚的量是 E3（首次可见）到自车到达的时间差，以及 LEAD 在 E3 "
  "之后有没有持续制动。被遮挡这件事本身已经用几何测过：释放前行人身体被挡 "
  "0.89，且完全在 LEAD 前视相机视场内。")
A("")
A("")

# ------------------------------------------------------------------- why v4
A("## 为什么重做场景")
A("")
A("| v3 被指出的问题 | v4 的处理 | 证据 |")
A("|---|---|---|")
A("| 高速式双车道、无人行道 | **Town05_Opt 第 19 号路 −1 车道**：城市街道，"
  "右侧依次为 Parking / Shoulder / Sidewalk 车道 | 下方车道剖面 |")
A("| 遮挡车硬编码 z=1.0 悬浮 | **Nissan Patrol SUV 停入真实 Parking 车道**，"
  "按包围盒落在路面 | 底面与路面差 **+0.008 m** |")
A("| 行人 `set_transform` 滑动、身体朝向与运动方向垂直 | **物理行人 + "
  "`carla.WalkerControl`，朝向沿行走方向** | 实测 `walker_mode=physics`，"
  "9.0 m 横穿按 1.5 m/s 走出 |")
A("| 遮挡只是假设 | 逐帧计算**行人身体 15 个采样点被车身挡住的比例**，"
  "视点用 LEAD 前视相机 | 遮挡 0.89，首次可见 1.6 s |")
A("| 4 fps 低清拼接、播放速度失真 | 每个仿真 tick 采一帧（20 Hz），"
  "按 `image.frame` 重排并补帧，按 20 fps 编码 | 帧清单 gaps/cover |")
A("| 视频不是 LEAD 的输入视角 | 主视角就是 **LEAD 前视相机本体**"
  "（0.25, 0, 2.25, 384×384, fov 60） | 演示视频左下格 |")
A("| 无事件模型 | trace 推出 **E1–E5**，并把 LEAD 自身感知按帧对齐 | 事件表 |")
A("")

# ------------------------------------------------------------------ geometry
A("## 场景几何（可复现的自检）")
A("")
A("`scripts/verify_geometry_v4.py` 只连 CARLA、不跑 LEAD，复现场景的放置规则并"
  "打印车道剖面、贴地误差与「行人身体从 LEAD 前视相机看还剩多少可见」。"
  "完整输出见 `ghost_pedestrian_v4/results/geometry_check.txt`：")
A("")
if geom:
    A("```text")
    A(geom)
    A("```")
    A("")
A("**关键设计取舍**：行人的起点放在**车头外侧**（横向 4.0 m，Parking 车道外沿"
  "的可行走带）而不是人行道中心（5.4 m）。上表是原因：人行道中心的行人在 12 m "
  "以外就已经完全可见（可见度 0.20→1.00），根本构不成「被遮挡」；只有紧贴车头"
  "外侧的 4.0 m 处，行人在 8–40 m 全程可见度都是 **0.00**，才真正复现鬼探头。"
  "这是用几何测出来的，不是摆出来的。")
A("")
A("遮挡车与行人的相对位置也由包围盒推导：SUV 车头停在横穿线前 1.2 m"
  "（`occluder_nose`），车长 4.60 m，于是车身中心在横穿线前 3.50 m；"
  "行人站在车头外侧 1.2 m 处的可行走带。")
A("")
A("遮挡判定使用 **LEAD 的前视相机**（sensor rig `rgb_2`：位置 (0.25, 0, 2.25)，"
  "384×384，fov 60），不是演示相机，所以「看不见」指的是模型真实输入看不见。"
  "每个采样点同时检查**是否在视锥内**与**射线是否穿过车身 OBB**，两者任一不满足"
  "都算不可见；遮挡率 = 1 − 可见采样点 / 15。")
A("")

# --------------------------------------------------------------------- sweep
A("## 距离扫描（LEAD，行人 1.5 m/s）")
A("")
if sweep:
    A("触发距离定义为「释放瞬间自车到横穿线的距离」，实测误差在 ±0.02 m 内。")
    A("")
    A("| 触发 (m) | 实测释放 (m) | 碰撞 | 最小距离 (m) | 遮挡均值 | 首次可见 (s) | 进车道 (s) | 持续制动 (s) |")
    A("|---|---|---|---|---|---|---|---|")
    for r in sweep:
        A("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            fmt(r["trigger_m"], 0), fmt(r["actual_m"], 3), yes(r["collision"]),
            fmt(r["min_dist_m"], 3), fmt(r["occ_before"]),
            fmt(r["visible_s"]), fmt(r["lane_entry_s"]), fmt(r["brake_s"])))
    A("")
    A("三个时刻的竞争（同一批运行的 `进车道` 与 `自车到达`）：")
    A("")
    A("| 触发 (m) | 行人进车道 (s) | 自车到横穿线 (s) | 最小距离 (m) | 结果 |")
    A("|---|---|---|---|---|")
    for r in sweep:
        A("| %s | %s | %s | %s | %s |" % (
            fmt(r["trigger_m"], 0), fmt(r["lane_entry_s"]),
            fmt(r["ego_cross_s"]), fmt(r["min_dist_m"], 3),
            "碰撞" if r["collision"] == "True" else "安全"))
    A("")
    short = [r for r in safe if r.get("ego_cross_s") and r.get("lane_entry_s")
             and float(r["ego_cross_s"]) < float(r["lane_entry_s"])]
    if short:
        A("触发 ≤ %s m 的档位里，自车到达横穿线（%s–%s s）早于行人进车道"
          "（%s s）——行人还没走进车道，车就已经过去了，所以安全。" % (
              fmt(max(float(r["trigger_m"]) for r in short), 0),
              fmt(min(float(r["ego_cross_s"]) for r in short), 2),
              fmt(max(float(r["ego_cross_s"]) for r in short), 2),
              fmt(min(float(r["lane_entry_s"]) for r in short), 2)))
    if coll:
        c = coll[0]
        try:
            delta = float(c["ego_cross_s"]) - float(c["lane_entry_s"])
            A("最短的碰撞档 %s m：行人进车道 %s s、自车到横穿线 %s s，"
              "只差 %.2f s，行人恰好在自车到达前进入车道。" % (
                  fmt(c["trigger_m"], 0), fmt(c["lane_entry_s"]),
                  fmt(c["ego_cross_s"]), delta))
        except Exception:
            pass
    A("")
    A("**不要把单个距离点的碰撞/安全当成确定边界**：20 m 三次重复里 2 次碰撞"
      "（见下一节），边界附近的单次结果有随机性——LEAD 的辅助行人感知有时能"
      "升到 0.4、有时停在 0.06（见「LEAD 自己的感知」），直接决定它是否制动。"
      "结论要按重复率和 E3→E4 的时序说。")
    A("")
else:
    A("（尚无扫描结果。）")
    A("")

# ------------------------------------------------------------------- repeats
A("## 边界可重复性")
A("")
if repeats:
    A("同一路段、同一触发距离重复 3 次，用来看「边界」是否稳定（而不是单次运气）。")
    A("")
    A("| 触发 (m) | 次数 | 碰撞次数 | 碰撞率 | 最小距离均值 (m) | 范围 (m) | 各次实测释放 (m) |")
    A("|---|---|---|---|---|---|---|")
    for r in repeats:
        A("| %s | %s | %s | %s | %s | %s–%s | %s |" % (
            fmt(r["trigger_m"], 0), r["runs"], r["collisions"],
            "%.0f%%" % (float(r["collision_rate"]) * 100),
            fmt(r["min_dist_mean"], 3), fmt(r["min_dist_min"], 3),
            fmt(r["min_dist_max"], 3), r["egos"]))
    A("")
else:
    A("（尚无重复实验。）")
    A("")

# ------------------------------------------------------------------ controls
A("## 对照实验、Oracle 与行人速度")
A("")
if controls:
    A("| 运行 | 触发 (m) | 碰撞 | 最小距离 (m) | 遮挡均值 | 首次可见 (s) | 持续制动 (s) | Oracle |")
    A("|---|---|---|---|---|---|---|---|")
    for r in controls:
        A("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["run"], fmt(r["trigger_m"], 0), yes(r["collision"]),
            fmt(r["min_dist_m"], 3), fmt(r["occ_before"]), fmt(r["visible_s"]),
            fmt(r["brake_s"]), r["oracle"] or "—"))
    A("")
    A("- **无遮挡对照**（`v4_no_occluder`，有行人无 SUV）：行人在接近过程中一直"
      "可见，用来分离「遮挡」与「横穿本身」的贡献。")
    no_occ = next((r for r in controls if r["run"] == "v4_no_occluder"), None)
    occ20 = next((r for r in controls
                  if r["run"] == "v4_d20m"), None)
    if (no_occ and occ20 and no_occ.get("visible_s")
            and occ20.get("visible_s")):
        A("- 同一 20 m 触发：**无遮挡时首次可见在释放后 %s s，有遮挡时 %s s**，"
          "遮挡吃掉了 %.2f s 的反应窗口。两者%s，说明 20 m 本身已经贴着 LEAD 的"
          "反应极限，遮挡是在这个已经很紧的窗口上再削掉 1.5 s，而不是唯一原因。"
          "所以结论要按 E3 的可用时间讲。" % (
              fmt(no_occ["visible_s"]), fmt(occ20["visible_s"]),
              float(occ20["visible_s"]) - float(no_occ["visible_s"]),
              "都碰撞" if (no_occ["collision"] == "True"
                          and occ20["collision"] == "True") else "结果不同"))
    A("- **仅遮挡车**（`v4_occluder_only`，无行人）：场景基线，证明碰撞不是撞车。")
    A("- 固定 20 m 换行人速度：1.0 m/s 与 2.0 m/s 都安全、1.5 m/s 碰撞——速度不是"
      "单调关系，改变的是「行人占据车道的时刻」与自车到达时刻的相对关系。")
    A("- **Oracle**：在 20 m 触发（LEAD 碰撞的那一档）用外部全制动替代模型决策。"
      "首次可见即刹（`or_visible`）说明物理空间是否够；若 Oracle 安全而 LEAD 撞，"
      "瓶颈在决策时机而不在车速或路面。")
    A("")
else:
    A("（尚无对照结果。）")
    A("")

# -------------------------------------------------------------------- events
A("## 首次可见时还有多少空间")
A("")
if physics:
    A("把「模型看见得太晚」和「物理上来不及」分开：取 E3（首次可见）那一刻的"
      "车速与车-人距离，与碰撞前实测到的最大减速度比较。`刹停所需 = v² / 2a`。"
      "注意 summary 里的 `max_decel` 含碰撞冲量，这里只统计碰撞前。")
    A("")
    A("| 运行 | 碰撞 | E3 车速 (m/s) | E3 距离 (m) | 碰撞前最大减速度 (m/s²) | 刹停所需 (m) | 余量 (m) |")
    A("|---|---|---|---|---|---|---|")
    for r in physics:
        if not r["dist_at_visible_m"]:
            continue
        A("| %s | %s | %s | %s | %s | %s | %s |" % (
            r["run"], yes(r["collision"]), fmt(r["v_at_visible_mps"]),
            fmt(r["dist_at_visible_m"], 3), fmt(r["max_decel_pre_crash"]),
            fmt(r["required_stop_m"]), fmt(r["margin_m"])))
    A("")
    A("余量为正表示「看见之后其实来得及刹住」，失效来自响应时机/反应延迟，"
      "而不是车速太高或刹车不够。Oracle 那一档的对照直接验证这一点。")
    A("")
else:
    A("（尚无数据。）")
    A("")

A("## 事件模型 E1–E5")
A("")
A("E1 被遮挡 → E2 释放 → E3 首次可见 → E4 自车持续制动 → E5 结果。"
  "E4 可以缺失（模型没有响应也是有效结果）。")
A("")
if events:
    A("| 运行 | E1 遮挡 | E2 释放 (s) | E3 首次可见 (s) | E4 持续制动 (s) | E5 结果 | 最小距离 (m) |")
    A("|---|---|---|---|---|---|---|")
    for name in ("v4_d12m", "v4_d16m", "v4_d20m", "v4_d24m", "v4_no_occluder",
                 "v4_occluder_only", "v4_d20m_or_visible",
                 "v4_boundary20_r1", "v4_boundary24_r1"):
        e = events.get(name)
        if not e:
            continue
        A("| %s | %s | %s | %s | %s | %s | %s |" % (
            name, fmt(e.get("E1_occ_before")), fmt(e.get("E2_release_s")),
            fmt(e.get("E3_visible_s")), fmt(e.get("E4_brake_s")),
            e.get("E5_outcome"), fmt(e.get("min_dist_m"), 3)))
    A("")
else:
    A("（尚无事件表。）")
    A("")

# -------------------------------------------------------------- perception
A("## LEAD 自己的感知怎么说")
A("")
if percept:
    A("用 LEAD 推理钩子（`GP_PRED_LOG`）记录同一仿真帧上模型的辅助感知输出，"
      "再按帧号与 trace 对齐。`walker_*` 是辅助检测头的行人最高置信度，"
      "`bev_*` 是 BEV 语义里行人类的最大值。")
    A("")
    A("| 运行 | 遮挡均值 | 释放前 walker | 释放后 walker | 释放前 BEV | 释放后 BEV |")
    A("|---|---|---|---|---|---|")
    for r in percept:
        A("| %s | %s | %s | %s | %s | %s |" % (
            r["run"], fmt(r["occ_before"]),
            fmt(r["lead_walker_max_before"], 3), fmt(r["lead_walker_max_after"], 3),
            fmt(r["lead_bev_max_before"], 3), fmt(r["lead_bev_max_after"], 3)))
    A("")
    A("这张表的用途是把「几何上看不见」和「模型确实没认出来」对上：被遮挡期间"
      "辅助感知接近 0，进入视场后仍可能很低——说明 LEAD 的失效并不只是「看得见"
      "但没反应过来」，而是**看见得太晚、可用信息太少**。但它只是辅助头，"
      "不能替代对规划输出的分析；E4 的制动时序才是模型行为的直接证据。")
    A("")
else:
    A("（尚无感知对齐数据；需要带 `pred` 的运行。）")
    A("")

# -------------------------------------------------------------------- videos
A("## 视频与录制")
A("")
A("每个演示四格：左上追尾视角、右上俯视、左下**LEAD 前视相机本体**、"
  "右下逐帧 HUD（时间、车速、制动、行人被遮挡比例、LEAD 行人置信度）。")
A("")
if frames:
    A("| 运行 | 帧数与丢帧 |")
    A("|---|---|")
    for r in frames:
        parts = []
        for t in ("lead", "top", "chase"):
            if r.get(t):
                parts.append("%s n=%s gaps=%s cover=%s" % (
                    t, r[t], r.get(t + "_gaps"), r.get(t + "_cover")))
        if parts:
            A("| %s | %s |" % (r["run"], "；".join(parts)))
    A("")
A("帧按 `image.frame` 排序，缺失帧用上一帧补，保证播放时间轴与仿真 20 Hz "
  "一致（不会出现 v3 那种「4 fps 素材按 10 fps 播」的加速）。")
A("")
A("演示文件：`ghost_pedestrian_v4/demos/`（`demo_collision.mp4`、"
  "`demo_safe.mp4`、`demo_oracle.mp4`、`demo_occluder_only.mp4`、"
  "`demo_comparison.mp4`）。")
A("")
A("画质保持 `Low`：审阅意见明确指出 High 会改变 LEAD 的视觉输入分布，"
  "所以不能只在演示时开 High；且本无头 Vulkan 构建在 High 下更易崩。"
  "正式实验与演示使用同一画质。")
A("")

# ------------------------------------------------------------------ repro
A("## 复现")
A("")
A("```bash")
A("# 一次跑完（几何自检 → 实验 → 演示 → 分析 → 文档 → 视频）")
A("bash /root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4/scripts/run_all_v4.sh")
A("")
A("# 或者分步：")
A("conda activate /root/autodl-tmp/envs/lead")
A("bash /root/autodl-tmp/v4_set_map.sh Town05_Opt          # 启动前定图，评估中不 load_world")
A("python3 ghost_pedestrian_v4/scripts/gen_routes_v4.py")
A("bash ghost_pedestrian_v4/scripts/install_v4.sh")
A("bash /root/autodl-tmp/v4_start_carla.sh 2000 Low")
A("python3 ghost_pedestrian_v4/scripts/verify_geometry_v4.py > ghost_pedestrian_v4/results/geometry_check.txt")
A("bash ghost_pedestrian_v4/scripts/run_v4.sh /tmp/v4_sweep.txt 420")
A("python3 ghost_pedestrian_v4/scripts/analyze_v4.py")
A("python3 ghost_pedestrian_v4/scripts/gen_v4_doc.py")
A("python3 ghost_pedestrian_v4/scripts/make_demos_v4.py")
A("```")
A("")
A("`run_v4.sh` 每条路线都重启 CARLA；部分运行返回码 250 是因为无头构建在"
  "**写完 summary 之后**退出时 abort，分析脚本以 `summary.status == \"final\"` "
  "作为成功判据。")
A("")

# ------------------------------------------------------------------ limits
A("## 下一步（V4.3）")
A("")
A("1. **跨路段/跨遮挡物**：同图换一个路段，或把遮挡物换成更高的厢式车/路口转角，"
  "验证「遮挡吃掉 1.5 s 反应窗口」不是这一段路的偶然。")
A("2. **危险参数生成**：把 E1→E2→E3 的时序约束写成可搜索的目标（例如「要求在自车"
  "到达前 1.0–1.5 s 才进入车道」），而不是只扫触发距离；这才是 v3 里 P6 想做的事。")
A("3. **传感器级对齐**：记录 LEAD 三路前视相机各自的传感器帧号，补偿 1–2 tick 的"
  "感知延迟，把「模型什么时候拿到信息」和「模型什么时候反应」彻底分开。")
A("4. **更细的遮挡度量**：用语义分割/深度图算像素级遮挡比例，替代 15 点采样。")
A("")

A("## 已知限制")
A("")
A("- 行人行走是 `WalkerControl` 物理驱动（实测有效）：全部运行里「物理失效→"
  "确定性路径」的回退一次都没有触发；个别运行的 `walker_mode=unknown` 只是"
  "行人还没走完、运行先结束了。CARLA 的行人步态是程序化动画，不是动作捕捉。")
A("- 遮挡率用 15 个身体采样点近似，未做像素级或语义分割级遮挡。")
A("- LEAD 的传感器帧号与推理帧号相差 1–2 tick；本文的时间都相对「释放 tick」"
  "并按仿真帧对齐，未单独补偿传感器延迟。")
A("- 只做了 Town05_Opt 的一个路段；不同路段的结论需要另行验证（v4.3）。")
A("- 场景危险参数只扫了触发距离与行人速度；遮挡位置、行人起点的系统扫描留作"
  "后续的危险场景生成。")
A("")

with open(OUT, "w") as f:
    f.write("\n".join(L) + "\n")
print("wrote", OUT, len(L), "lines")
