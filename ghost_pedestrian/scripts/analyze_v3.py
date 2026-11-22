#!/usr/bin/env python3
"""v3 analysis: P4 release-state distribution, P3.5 model timeline, P5 2-D map."""
import csv
import json
import os
import statistics

RUNS = "/root/autodl-tmp/ghost_runs"
RESULTS = "/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/results"
os.makedirs(RESULTS, exist_ok=True)


def load(name):
    p = os.path.join(RUNS, name, "summary.json")
    if not os.path.exists(p):
        return None
    try:
        return json.load(open(p))
    except Exception:
        return None


def trace(name):
    p = os.path.join(RUNS, name, "trace.jsonl")
    if not os.path.exists(p):
        return []
    out = []
    with open(p) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


def fnum(x):
    return None if x in (None, "") else round(float(x), 3)


def p4():
    rows = []
    for name in sorted(os.listdir(RUNS)):
        if not name.startswith("p4_"):
            continue
        s = load(name)
        if not s or s.get("status") != "final":
            continue
        r = {
            "run": name,
            "nominal_m": s.get("trigger_distance_m"),
            "actual_m": fnum(s.get("actual_trigger_distance_m")),
            "post_m": fnum(s.get("first_post_release_distance_m")),
            "tick_offset_m": (round(s["actual_trigger_distance_m"]
                                    - s["first_post_release_distance_m"], 3)
                              if s.get("actual_trigger_distance_m") is not None
                              and s.get("first_post_release_distance_m") is not None
                              else None),
            "speed": fnum(s.get("ego_speed_at_release_mps")),
            "collision": bool(s.get("collision_pedestrian")),
            "min_dist_m": fnum(s.get("min_distance_m")),
            "brake_rel_s": fnum(s.get("brake_onset_after_release_s")),
            "frame": s.get("release_frame"),
        }
        rows.append(r)
        # Physics-informed effective margin from the measured release state:
        # ego front arrival vs pedestrian clearing the ego lane (4.4 m dash).
        if (r["actual_m"] is not None and r["speed"]
                and s.get("walker_speed_mps")):
            arrival = (r["actual_m"] - 2.45) / r["speed"]
            clear = 4.4 / float(s["walker_speed_mps"])
            r["arrival_s"] = round(arrival, 3)
            r["clear_s"] = round(clear, 3)
            r["margin_s"] = round(arrival - clear, 3)
        else:
            r["arrival_s"] = r["clear_s"] = r["margin_s"] = None
    print("=== P4: RELEASE STATE BY NOMINAL DISTANCE (5 runs each) ===")
    print("%-20s %8s %8s %8s %7s %7s %8s %8s" % (
        "nominal", "n", "actual", "sd", "speed", "sd", "coll", "post"))
    for nominal in sorted({r["nominal_m"] for r in rows}):
        g = [r for r in rows if r["nominal_m"] == nominal]
        if not g:
            continue
        act = [r["actual_m"] for r in g if r["actual_m"] is not None]
        spd = [r["speed"] for r in g if r["speed"] is not None]
        print("%-20s %8d %8.2f %8.3f %7.2f %7.3f %8s %8.2f" % (
            nominal, len(g), statistics.mean(act) if act else -1,
            statistics.pstdev(act) if len(act) > 1 else 0,
            statistics.mean(spd) if spd else -1,
            statistics.pstdev(spd) if len(spd) > 1 else 0,
            "%d/%d" % (sum(r["collision"] for r in g), len(g)),
            statistics.mean([r["post_m"] for r in g if r["post_m"] is not None])))
    print()
    print("%-22s %9s %9s %8s %9s %8s %9s" % (
        "run", "nominal", "actual", "speed", "tickOff", "coll", "minDist"))
    for r in rows:
        print("%-22s %9s %9s %8s %9s %8s %9s" % (
            r["run"], r["nominal_m"], r["actual_m"], r["speed"],
            r["tick_offset_m"], r["collision"], r["min_dist_m"]))
    # Mixed-outcome comparison at the same nominal distance.
    print("\n--- effective time margin (arrival - lane clear), by outcome ---")
    for lbl, subset in (("SAFE", [r for r in rows if not r["collision"]]),
                        ("COLLISION", [r for r in rows if r["collision"]])):
        vals = [r["margin_s"] for r in subset if r["margin_s"] is not None]
        if vals:
            print("%-10s n=%2d mean=%6.3f sd=%6.3f range=[%.3f, %.3f]" % (
                lbl, len(vals), statistics.mean(vals),
                statistics.pstdev(vals) if len(vals) > 1 else 0,
                min(vals), max(vals)))
    print("\n--- same-nominal mixed outcomes ---")
    for nominal in sorted({r["nominal_m"] for r in rows}):
        g = [r for r in rows if r["nominal_m"] == nominal]
        safe = [r for r in g if not r["collision"]]
        coll = [r for r in g if r["collision"]]
        if safe and coll:
            print("nominal=%s: safe n=%d actual=%.2f speed=%.2f | "
                  "collision n=%d actual=%.2f speed=%.2f" % (
                      nominal, len(safe),
                      statistics.mean([r["actual_m"] for r in safe]),
                      statistics.mean([r["speed"] for r in safe]),
                      len(coll),
                      statistics.mean([r["actual_m"] for r in coll]),
                      statistics.mean([r["speed"] for r in coll])))
    with open(os.path.join(RESULTS, "p4_release_states.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]) if rows else [])
        w.writeheader()
        w.writerows(rows)
    return rows


def closest(records, t_rel):
    if not records:
        return None
    return min(records, key=lambda r: abs(r["t_rel"] - t_rel))


def p35():
    print("\n=== P3.5: LEAD INTERNAL TIMELINE (paired 24 m runs) ===")
    times = [0.50, 0.70, 0.90, 1.20, 1.50]
    out = {}
    for name in ("p35_lead_24m", "p35_or_24m"):
        s = load(name)
        if not s or s.get("status") != "final":
            print("  %s: no final summary" % name)
            continue
        t0 = s["release_time_s"]
        pred = []
        pp = os.path.join(RUNS, name, "pred.jsonl")
        if os.path.exists(pp):
            for line in open(pp):
                try:
                    rec = json.loads(line)
                    rec["t_rel"] = rec["t"] - t0
                    pred.append(rec)
                except Exception:
                    pass
        tr = trace(name)
        for r in tr:
            r["t_rel"] = r["t"] - t0
        print("\n--- %s (%d model steps, %d trace records) ---" % (
            name, len(pred), len(tr)))
        print("%6s %10s %10s %10s %10s %10s %9s %8s" % (
            "t_rel", "occluded", "walkerY", "boxWmax", "bevWmax",
            "tgtSpdPost", "brake", "control"))
        tab = []
        for tt in times:
            p = closest(pred, tt)
            g = closest(tr, tt)
            if not p:
                continue
            row = {
                "t_rel": tt,
                "actual_t": round(p["t_rel"], 3),
                "frame": p.get("frame"),
                "occluded": g.get("occluded") if g else None,
                "walker_y": g.get("walker_y") if g else None,
                "box_walker_max": p.get("walker_score_max"),
                "bev_walker_max": p.get("bev_walker_max"),
                "sem_ped_max": p.get("sem_ped_max"),
                "target_speed_scalar": p.get("target_speed_scalar"),
                "target_speed_post": p.get("target_speed_post"),
                "prob_bin0": p.get("target_speed_prob_bin0"),
                "boxes": p.get("n_boxes"),
                "brake": g.get("brake") if g else None,
                "control_brake": p["control"][2] if p.get("control") else None,
            }
            tab.append(row)
            print("%6.2f %10s %10s %10s %10s %10s %9s %8s" % (
                tt, row["occluded"], row["walker_y"], row["box_walker_max"],
                row["bev_walker_max"], row["target_speed_post"],
                row["brake"], row["control_brake"]))
        out[name] = tab
    # First-detection and target-speed-drop times from the full pred stream.
    for name, tab in list(out.items()):
        pp = os.path.join(RUNS, name, "pred.jsonl")
        if not os.path.exists(pp):
            continue
        recs = [json.loads(l) for l in open(pp)]
        t0 = load(name)["release_time_s"]
        for r in recs:
            r["t_rel"] = r["t"] - t0
        det = next((r for r in recs if (r.get("walker_score_max") or 0) > 0.15
                    or (r.get("bev_walker_max") or 0) > 0.15), None)
        v0 = next((r["target_speed_post"] for r in recs
                   if r.get("target_speed_post") is not None), None)
        drop = None
        if v0:
            drop = next((r for r in recs
                         if r.get("target_speed_post") is not None
                         and r["target_speed_post"] < 0.5 * v0), None)
        print("%s: first_aux_detection=%s s, target_speed_halved=%s s" % (
            name,
            round(det["t_rel"], 3) if det else None,
            round(drop["t_rel"], 3) if drop else None))
        out.setdefault("_meta", {})[name] = {
            "first_aux_detection_s": round(det["t_rel"], 3) if det else None,
            "target_speed_halved_s": round(drop["t_rel"], 3) if drop else None,
            "n_model_steps": len(recs),
        }
    with open(os.path.join(RESULTS, "p35_timeline.json"), "w") as fh:
        json.dump(out, fh, indent=2)


def p5():
    print("\n=== P5: WALKER SPEED x TRIGGER DISTANCE (2-D outcome map) ===")
    cells = {}
    for name in sorted(os.listdir(RUNS)):
        if not name.startswith("p5_"):
            continue
        s = load(name)
        if not s or s.get("status") != "final":
            continue
        parts = name.split("_")
        spd = parts[1][1:].rstrip("m").replace("p", ".")
        dist = parts[2][1:].rstrip("m").replace("p", ".")
        key = (float(spd), float(dist))
        cells[key] = ("COLL" if s.get("collision_pedestrian")
                      else "SAFE", fnum(s.get("min_distance_m")),
                      fnum(s.get("actual_trigger_distance_m")))
    speeds = sorted({k[0] for k in cells})
    dists = sorted({k[1] for k in cells})
    print("%10s " % "speed", " ".join("%12s" % ("%gm" % d) for d in dists))
    for sp in speeds:
        print("%10s " % sp, " ".join(
            "%12s" % (cells.get((sp, d), ("--", None, None))[0]) for d in dists))
    print("\nmin distance (m):")
    for sp in speeds:
        print("%10s " % sp, " ".join(
            "%12s" % cells.get((sp, d), ("--", None, None))[1] for d in dists))
    # Non-monotonicity check: for each speed, ordered by distance, count sign flips.
    for sp in speeds:
        seq = [cells[(sp, d)][0] == "COLL" for d in dists if (sp, d) in cells]
        flips = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        print("speed %.1f: collision sequence=%s, sign flips=%d" % (
            sp, ["1" if x else "0" for x in seq], flips))
    with open(os.path.join(RESULTS, "p5_map.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["walker_speed_mps", "trigger_m", "outcome",
                    "min_dist_m", "actual_trigger_m"])
        for k in sorted(cells):
            w.writerow([k[0], k[1], cells[k][0], cells[k][1], cells[k][2]])


if __name__ == "__main__":
    p4()
    p35()
    p5()
