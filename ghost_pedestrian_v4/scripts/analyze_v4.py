#!/usr/bin/env python3
"""Analyze GhostPedestrianV4 runs.

Outputs (ghost_pedestrian_v4/results/):
  v4_sweep.csv      distance sweep
  v4_controls.csv   baseline / occluder-only / no-occluder / oracle / speed
  v4_repeats.csv    boundary repeats
  v4_events.json    E1-E5 timeline per run
  v4_perception.csv LEAD auxiliary-perception alignment per run
  v4_frames.csv     frame continuity of the demo runs
"""
import csv
import json
import os
import statistics

RUNS = "/root/autodl-tmp/ghost_runs_v4"
RESULTS = "/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4/results"
os.makedirs(RESULTS, exist_ok=True)

SWEEP = [8, 12, 16, 20, 24, 26, 28, 30, 34, 40]
CONTROLS = ["v4_baseline", "v4_occluder_only", "v4_no_occluder"]
ORACLE = ["v4_d20m", "v4_d20m_or_immediate", "v4_d20m_or_visible"]
SPEED = ["v4_s1p0_d20m", "v4_d20m", "v4_s2p0_d20m"]
REPEATS = {"20": ["v4_boundary20_r%d" % i for i in (1, 2, 3)],
           "24": ["v4_boundary24_r%d" % i for i in (1, 2, 3)],
           "26": ["v4_boundary26_r%d" % i for i in (1, 2, 3)]}


def load(name):
    p = os.path.join(RUNS, name, "summary.json")
    if not os.path.exists(p):
        return None
    try:
        s = json.load(open(p))
    except Exception:
        return None
    return s if s.get("status") == "final" else None


def trace(name):
    p = os.path.join(RUNS, name, "trace.jsonl")
    out = []
    if not os.path.exists(p):
        return out
    for line in open(p):
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


def pred(name):
    p = os.path.join(RUNS, name, "pred.jsonl")
    out = []
    if not os.path.exists(p):
        return out
    for line in open(p):
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


def fnum(x, nd=2):
    try:
        return round(float(x), nd)
    except Exception:
        return None


def row(name, s):
    trig = fnum(s.get("trigger_distance_m"))
    if s.get("walker_speed_mps") is None:
        # no pedestrian in this run (occluder-only baseline): the trigger
        # distance has no meaning.
        trig = None
    return {
        "run": name,
        "trigger_m": trig,
        "actual_m": fnum(s.get("actual_trigger_distance_m"), 3),
        "speed_mps": fnum(s.get("release_speed_mps")),
        "collision": bool(s.get("collision_pedestrian")),
        "min_dist_m": fnum(s.get("min_distance_m"), 3),
        "occ_before": fnum(s.get("occlusion_mean_before_release"), 3),
        "in_fov_before": fnum(s.get("in_fov_mean_before_release"), 3),
        "visible_s": fnum(s.get("first_visible_after_release_s")),
        "lane_entry_s": fnum(s.get("ped_lane_entry_after_release_s")),
        "ego_cross_s": fnum(s.get("ego_cross_after_release_s")),
        "brake_s": fnum(s.get("sustained_brake_after_release_s")),
        "oracle": s.get("oracle_mode"),
        "walker_mode": s.get("walker_mode"),
        "town": s.get("town"),
        "records": s.get("num_records"),
    }


def write_csv(name, rows):
    with open(os.path.join(RESULTS, name), "w", newline="") as f:
        if not rows:
            f.write("")
            return
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def show(rows, title, cols):
    print("\n=== %s ===" % title)
    print("  %-22s %s" % ("run", " ".join("%-9s" % c for c in cols)))
    for r in rows:
        label = r.get("run")
        if label is None:
            label = "trigger %sm" % fnum(r.get("trigger_m"), 0)
        print("  %-22s %s" % (label, " ".join(
            "%-9s" % r.get(c) for c in cols)))


def sweep():
    rows = [row(n, s) for n in ["v4_d%dm" % d for d in SWEEP] for s in [load(n)] if s]
    rows.sort(key=lambda r: (r["trigger_m"] if r["trigger_m"] is not None else -1))
    show(rows, "DISTANCE SWEEP (Town05_Opt, walker 1.5 m/s)",
         ["trigger_m", "actual_m", "collision", "min_dist_m", "occ_before",
          "visible_s", "lane_entry_s", "brake_s"])
    coll = [r["trigger_m"] for r in rows if r["collision"]]
    safe = [r["trigger_m"] for r in rows if not r["collision"]]
    print("  collision:", coll, " safe:", safe)
    write_csv("v4_sweep.csv", rows)
    return rows


def controls():
    names = list(dict.fromkeys(CONTROLS + ORACLE + SPEED))
    rows = [row(n, s) for n in names for s in [load(n)] if s]
    show(rows, "CONTROLS / ORACLE / WALKER SPEED",
         ["trigger_m", "collision", "min_dist_m", "occ_before", "visible_s",
          "brake_s", "oracle"])
    write_csv("v4_controls.csv", rows)
    return rows


def repeats():
    rows = []
    for d, names in sorted(REPEATS.items()):
        got = [row(n, s) for n in names for s in [load(n)] if s]
        if not got:
            continue
        n_coll = sum(1 for r in got if r["collision"])
        mins = [r["min_dist_m"] for r in got if r["min_dist_m"] is not None]
        rows.append({
            "trigger_m": float(d),
            "runs": len(got),
            "collisions": n_coll,
            "collision_rate": round(n_coll / float(len(got)), 3),
            "min_dist_mean": round(statistics.mean(mins), 3) if mins else None,
            "min_dist_min": min(mins) if mins else None,
            "min_dist_max": max(mins) if mins else None,
            "egos": ",".join("%.2f" % r["actual_m"] for r in got
                             if r["actual_m"] is not None),
        })
    show(rows, "BOUNDARY REPEATS",
         ["trigger_m", "runs", "collisions", "collision_rate", "min_dist_mean",
          "min_dist_min", "min_dist_max"])
    write_csv("v4_repeats.csv", rows)
    return rows


def events():
    out = {}
    for name in sorted(os.listdir(RUNS)):
        s = load(name)
        if not s:
            continue
        tr = trace(name)
        rel = [r for r in tr if r.get("released")]
        if not rel:
            continue
        t0 = rel[0]["t"]
        pre = [r for r in tr if not r.get("released")
               and r.get("occlusion") is not None]
        vis = next((r for r in rel if r.get("occlusion") is not None
                    and r["occlusion"] < 0.5), None)
        brake = None
        for i in range(len(rel) - 2):
            if all(rel[i + j].get("brake", 0) > 0.3 for j in range(3)):
                brake = rel[i]
                break
        out[name] = {
            "E1_occ_before": (round(statistics.mean(
                [r["occlusion"] for r in pre]), 3) if pre else None),
            "E2_release_s": round(t0, 3),
            "E3_visible_s": round(vis["t"] - t0, 3) if vis else None,
            "E4_brake_s": round(brake["t"] - t0, 3) if brake else None,
            "E5_outcome": ("collision" if s.get("collision_pedestrian")
                           else "safe"),
            "min_dist_m": fnum(s.get("min_distance_m"), 3),
            "walker_mode": s.get("walker_mode"),
        }
    with open(os.path.join(RESULTS, "v4_events.json"), "w") as f:
        json.dump(out, f, indent=2)
    show([dict(run=k, **v) for k, v in sorted(out.items())][:20],
         "EVENT TIMELINE E1-E5",
         ["E1_occ_before", "E3_visible_s", "E4_brake_s", "E5_outcome",
          "min_dist_m"])
    return out


def perception():
    """Align LEAD's own auxiliary perception with the occlusion timeline."""
    rows = []
    for name in sorted(os.listdir(RUNS)):
        s = load(name)
        if not s:
            continue
        pr = pred(name)
        if not pr:
            continue
        tr = {r["frame"]: r for r in trace(name) if r.get("frame") is not None}
        rel = [r for r in trace(name) if r.get("released")]
        if not rel:
            continue
        release_frame = rel[0].get("frame")
        before, after = [], []
        for p in pr:
            f = p.get("frame")
            if f is None or f not in tr:
                continue
            score = p.get("walker_score_max")
            if score is None:
                continue
            item = (score, p.get("bev_walker_max") or 0.0,
                    tr[f].get("occlusion"))
            if release_frame is None or f < release_frame:
                before.append(item)
            else:
                after.append(item)
        if not before and not after:
            continue
        rows.append({
            "run": name,
            "n_pred": len(pr),
            "lead_walker_max_before": (round(max(x[0] for x in before), 3)
                                       if before else None),
            "lead_bev_max_before": (round(max(x[1] for x in before), 3)
                                    if before else None),
            "lead_walker_max_after": (round(max(x[0] for x in after), 3)
                                      if after else None),
            "lead_bev_max_after": (round(max(x[1] for x in after), 3)
                                   if after else None),
            "occ_before": fnum(s.get("occlusion_mean_before_release"), 3),
            "visible_s": fnum(s.get("first_visible_after_release_s")),
        })
    show(rows, "LEAD AUXILIARY PERCEPTION vs OCCLUSION",
         ["lead_walker_max_before", "lead_walker_max_after",
          "lead_bev_max_before", "lead_bev_max_after"])
    write_csv("v4_perception.csv", rows)
    return rows


def physics():
    """Was there room once the pedestrian became visible?

    Uses the trace only: at E3 (first visible) take the ego speed and the
    ego-pedestrian distance, and the largest deceleration actually reached
    *before* the collision impulse (the summary's max_decel is polluted by the
    impact). required = v^2 / (2a); margin = distance - required.
    """
    rows = []
    for name in sorted(os.listdir(RUNS)):
        s = load(name)
        if not s:
            continue
        tr = trace(name)
        rel = [r for r in tr if r.get("released")]
        if not rel:
            continue
        vis = next((r for r in rel if r.get("occlusion") is not None
                    and r["occlusion"] < 0.5), None)
        if not vis:
            continue
        hit = [c["t"] for c in (s.get("collisions_all") or [])
               if "walker" in c.get("other", "")]
        t_hit = min(hit) if hit else None
        decels = [r["decel"] for r in tr
                  if r.get("decel") is not None and 0.0 <= r["decel"] <= 12.0
                  and (t_hit is None or r["t"] < t_hit)]
        a = max(decels) if decels else None
        v = vis.get("ego_speed")
        d = vis.get("dist_ego_walker")
        req = (v * v / (2.0 * a)) if (v and a) else None
        rows.append({
            "run": name,
            "collision": bool(s.get("collision_pedestrian")),
            "v_at_visible_mps": fnum(v),
            "dist_at_visible_m": fnum(d, 3),
            "max_decel_pre_crash": fnum(a, 2),
            "required_stop_m": fnum(req, 2),
            "margin_m": (fnum(d - req, 2) if (d is not None and req is not None)
                         else None),
        })
    show(rows, "SPACE AT FIRST VISIBILITY (E3)",
         ["collision", "v_at_visible_mps", "dist_at_visible_m",
          "max_decel_pre_crash", "required_stop_m", "margin_m"])
    write_csv("v4_physics.csv", rows)
    return rows


def frames():
    rows = []
    print("\n=== FRAME CONTINUITY (demo runs) ===")
    for name in sorted(os.listdir(RUNS)):
        mpath = os.path.join(RUNS, name, "frames_manifest.jsonl")
        if not os.path.exists(mpath):
            continue
        by = {}
        for line in open(mpath):
            try:
                rec = json.loads(line)
            except Exception:
                continue
            by.setdefault(rec["tag"], []).append(int(rec["frame"]))
        if not by:
            continue
        row_ = {"run": name}
        for tag, ids in sorted(by.items()):
            ids = sorted(set(ids))
            gaps = sum(1 for a, b in zip(ids, ids[1:]) if b - a > 1)
            span = ids[-1] - ids[0] + 1 if ids else 0
            row_[tag] = len(ids)
            row_[tag + "_gaps"] = gaps
            row_[tag + "_cover"] = round(len(ids) / float(max(1, span)), 3)
        rows.append(row_)
        parts = []
        for tag in sorted(by):
            parts.append("%s n=%d gaps=%d cover=%.2f"
                         % (tag, row_[tag], row_[tag + "_gaps"],
                            row_[tag + "_cover"]))
        print("  %-20s %s" % (name, "  ".join(parts)))
    write_csv("v4_frames.csv", rows)
    return rows


if __name__ == "__main__":
    sweep()
    controls()
    repeats()
    events()
    perception()
    physics()
    frames()
    print("\nwrote results to", RESULTS)
