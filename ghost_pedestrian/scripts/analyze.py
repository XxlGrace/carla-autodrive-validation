#!/usr/bin/env python3
"""Aggregate ghost-pedestrian runs: outcome scan + mechanism diagnostics.

Discovers every gp_*m run under /root/autodl-tmp/ghost_runs, so extending the
distance grid in gen_routes.py needs no change here.
"""
import csv
import json
import os
import re

RUNS = "/root/autodl-tmp/ghost_runs"
HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "results")
os.makedirs(RESULTS, exist_ok=True)

CONTROLS = ["baseline", "occluder_only"]
def _trig_of(name):
    sp = os.path.join(RUNS, name, "summary.json")
    if os.path.exists(sp):
        v = json.load(open(sp)).get("trigger_distance_m")
        if v is not None:
            return float(v)
    mm = re.search(r"gp_(\d+(?:\.\d+)?)m", name)
    return float(mm.group(1)) if mm else None


GP = sorted(
    (d for d in os.listdir(RUNS)
     if d.startswith("gp_") and os.path.isdir(os.path.join(RUNS, d))
     and os.path.exists(os.path.join(RUNS, d, "summary.json"))),
    key=lambda s: (-_trig_of(s) if _trig_of(s) is not None else 1e9, s),
)
NAMES = CONTROLS + GP


def load(name):
    d = os.path.join(RUNS, name)
    sp = os.path.join(d, "summary.json")
    s = json.load(open(sp)) if os.path.exists(sp) else {}
    tp = os.path.join(d, "trace.jsonl")
    recs = [json.loads(l) for l in open(tp)] if os.path.exists(tp) else []
    log = os.path.join(RUNS, name + ".eval.log")
    crit = {}
    if os.path.exists(log):
        txt = re.sub(r"\x1b\[[0-9;]*[mK]", "", open(log, errors="ignore").read())
        for key in ["RouteCompletionTest", "CollisionTest", "MinSpeedTest",
                    "AgentBlockedTest"]:
            mm = re.search(key + r"\s*\u2502\s*(SUCCESS|FAILURE)", txt)
            if mm:
                crit[key] = mm.group(1)
    return s, recs, crit


def fnum(v, n=2):
    return None if v is None else round(float(v), n)


out = []
for name in NAMES:
    s, recs, crit = load(name)
    trig = _trig_of(name)
    wx = [r["walker_x"] for r in recs if r.get("walker_x") is not None]
    lex = [r["ego_x"] for r in recs if r.get("ego_x") is not None]
    crossed = bool(wx and min(wx) <= min(lex) + 0.9 and max(wx) >= max(lex) - 0.9)
    collision = s.get("collision_pedestrian", False)

    # Mechanism metrics derived from the trace (no scenario change required).
    rel = [r for r in recs if r.get("released")]
    post = [r for r in rel if r["t"] - rel[0]["t"] <= 3.0] if rel else []
    if post:
        t0 = rel[0]["t"]
        area = sum(post[i]["brake"] * (post[i + 1]["t"] - post[i]["t"])
                   for i in range(len(post) - 1))
        bticks = sum(1 for r in post if r.get("brake", 0) > 0.3)
        amin = min(r["ego_speed"] for r in post)
        clear = next((r for r in rel if r.get("ped_lateral") is not None
                      and r["ped_lateral"] < -1.2), None)
        clear_t = round(clear["t"] - t0, 3) if clear else None
        avail = s.get("available_dist_at_release_m")
        req_v = (round((avail - 2.45) / clear_t, 2)
                 if avail and clear_t and clear_t > 0 else None)
    else:
        area = bticks = amin = clear_t = req_v = None
    out.append({
        "brake_effort_area_3s": fnum(area),
        "brake_ticks_3s": bticks,
        "min_speed_3s_mps": fnum(amin),
        "ped_lane_clear_after_release_s": fnum(clear_t),
        "required_mean_speed_to_yield_mps": req_v,
        "run": name,
        "trigger_m": trig,
        "status": s.get("status"),
        "collision_ped": collision,
        "min_dist_m": fnum(s.get("min_distance_m")),
        "brake_onset_after_release_s": fnum(s.get("brake_onset_after_release_s")),
        "sustained_brake_after_release_s": fnum(s.get("sustained_brake_after_release_s")),
        "ego_speed_at_release_mps": fnum(s.get("ego_speed_at_release_mps")),
        "first_visible_after_release_s": fnum(s.get("first_visible_after_release_s")),
        "ped_lane_entry_after_release_s": fnum(s.get("ped_lane_entry_after_release_s")),
        "ego_cross_after_release_s": fnum(s.get("ego_cross_after_release_s")),
        "reaction_window_s": fnum(s.get("reaction_window_s")),
        "brake_latency_after_lane_entry_s": fnum(s.get("brake_latency_after_lane_entry_s")),
        "ego_speed_at_cross_mps": fnum(s.get("ego_speed_at_cross_mps")),
        "max_decel_mps2": fnum(s.get("max_decel_mps2")),
        "required_stop_dist_m": fnum(s.get("required_stop_dist_m")),
        "available_dist_at_release_m": fnum(s.get("available_dist_at_release_m")),
        "physical_margin_m": fnum(s.get("physical_margin_m")),
        "occluded_fraction_before_release": s.get("occluded_fraction_before_release"),
        "walker_crossed_ego_path": crossed,
        "criteria": crit,
        "outcome": ("COLLISION" if collision else
                    ("SAFE_AVOIDANCE" if s.get("sustained_brake_after_release_s") is not None
                     else "CONTROL")),
    })

with open(os.path.join(RESULTS, "summary.json"), "w") as fh:
    json.dump(out, fh, indent=2)

# CSV for plotting
cols = ["run", "trigger_m", "collision_ped", "min_dist_m",
        "sustained_brake_after_release_s", "reaction_window_s",
        "brake_latency_after_lane_entry_s", "ego_speed_at_cross_mps",
        "max_decel_mps2", "required_stop_dist_m", "physical_margin_m",
        "brake_effort_area_3s", "min_speed_3s_mps",
        "ped_lane_clear_after_release_s", "required_mean_speed_to_yield_mps",
        "outcome"]
with open(os.path.join(RESULTS, "scan.csv"), "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
    w.writeheader()
    for r in out:
        w.writerow(r)

print("=== OUTCOME SCAN (trigger distance sweep) ===")
print("%-12s %5s %9s %9s %9s %10s %s" % (
    "run", "trig", "collision", "minDist", "brakeRel", "speedCross", "outcome"))
for r in out:
    print("%-12s %5s %9s %9s %9s %10s %s" % (
        r["run"], r["trigger_m"], r["collision_ped"], r["min_dist_m"],
        r["sustained_brake_after_release_s"], r["ego_speed_at_cross_mps"],
        r["outcome"]))

print("\n=== MECHANISM DIAGNOSTICS (GP runs only) ===")
print("%-10s %8s %8s %8s %8s %8s %8s %8s %9s %8s" % (
    "trig", "visRel", "laneIn", "egoCross", "reactWin", "brakeRel", "brkLat",
    "spdCross", "maxDec", "margin"))
for r in out:
    if r["trigger_m"] is None:
        continue
    print("%-10s %8s %8s %8s %8s %8s %8s %8s %9s %8s" % (
        r["trigger_m"], r["first_visible_after_release_s"],
        r["ped_lane_entry_after_release_s"], r["ego_cross_after_release_s"],
        r["reaction_window_s"], r["sustained_brake_after_release_s"],
        r["brake_latency_after_lane_entry_s"], r["ego_speed_at_cross_mps"],
        r["max_decel_mps2"], r["physical_margin_m"]))

print("\n=== BRAKING EFFORT (3 s post-release, from trace) ===")
print("%-12s %6s %9s %9s %10s %9s %10s %s" % (
    "run", "trig", "brakeArea", "minSpeed", "reqMeanV", "laneClear", "margin", "outcome"))
for r in out:
    if r["trigger_m"] is None:
        continue
    print("%-12s %6s %9s %9s %10s %9s %10s %s" % (
        r["run"], r["trigger_m"], r["brake_effort_area_3s"],
        r["min_speed_3s_mps"], r["required_mean_speed_to_yield_mps"],
        r["ped_lane_clear_after_release_s"], r["physical_margin_m"],
        r["outcome"]))
