#!/usr/bin/env python3
"""Unified window analysis, anchored on the trigger distance (works for runs
without a pedestrian too).

Anchor tau=0 = first tick where the ego is within `trigger_distance` of the
crossing (identical to the release instant when a walker exists).
Window [-2, +5] s.

Streams are joined on the simulator frame id; the agent clock is mapped onto
the scenario clock (constant 1-tick offset observed, reported).
"""
import csv
import json
import os
import sys

A = sys.argv[1] if len(sys.argv) > 1 else "artifacts3/ghost_vad/results"
T_LO, T_HI = -2.0, 5.0


def load(tag):
    s = json.load(open(os.path.join(A, "summary_%s.json" % tag)))
    scen = [json.loads(l) for l in open(os.path.join(A, "scenario_trace_%s.jsonl" % tag))]
    vp = os.path.join(A, "vad_trace_%s.jsonl" % tag)
    vad = [json.loads(l) for l in open(vp)] if os.path.exists(vp) else []
    return s, scen, vad


def series(tag):
    s, scen, vad = load(tag)
    byf = {r["frame"]: r for r in scen if r.get("frame") is not None}
    trig = s.get("trigger_distance_m")
    # anchor: first tick inside the trigger distance
    anchor = None
    for r in scen:
        d = r.get("dist_ego_crossing")
        if d is not None and trig is not None and d <= trig + 1e-6:
            anchor = r["t"]
            break
    if anchor is None:
        anchor = s.get("release_time_s")
    if anchor is None:
        return None
    offs = sorted(byf[r["frame"]]["t"] - r["t"] for r in vad
                  if r.get("frame") in byf and byf[r["frame"]].get("t") is not None)
    off = offs[len(offs) // 2] if offs else 0.0
    out = []
    for r in vad:
        f = r.get("frame")
        if f not in byf:
            continue
        sr = byf[f]
        t = sr["t"] - anchor
        if T_LO <= t <= T_HI:
            out.append({"tau": t, "v": float(r.get("speed") or 0.0),
                        "des": float(r.get("desired_speed") or 0.0),
                        "brk": float(r.get("brake") or 0.0),
                        "thr": float(r.get("throttle") or 0.0),
                        "occ": sr.get("occlusion"),
                        "dist": sr.get("dist_ego_walker"),
                        "lat": sr.get("ped_lateral"),
                        "dcross": sr.get("dist_ego_crossing")})
    out.sort(key=lambda r: r["tau"])
    return s, out, off


def runs_of(seq, lo, hi, thr=0.3):
    seg = [r for r in seq if lo <= r["tau"] <= hi]
    res, cur = [], 0
    for r in seg:
        if r["brk"] > thr:
            cur += 1
        elif cur:
            res.append(cur)
            cur = 0
    if cur:
        res.append(cur)
    return res


def metrics(tag):
    got = series(tag)
    if not got:
        return None
    s, seq, off = got
    post = [r for r in seq if r["tau"] >= -0.01]
    w3 = [r for r in post if r["tau"] <= 3.0]
    w5 = [r for r in post if r["tau"] <= 5.0]
    if not w3:
        return None
    br = [r for r in w3 if r["brk"] > 0.3]
    rr = runs_of(seq, 0.0, 3.0)
    onset = None
    for i in range(len(w3) - 2):
        if all(w3[i + j]["brk"] > 0.3 for j in range(3)):
            onset = w3[i]["tau"]
            break
    vis = next((r["tau"] for r in post if r["occ"] is not None and r["occ"] < 0.5), None)
    near = min((r for r in post if r["dist"] is not None), key=lambda r: r["dist"], default=None)
    mv3 = min(w3, key=lambda r: r["v"])
    mv5 = min(w5, key=lambda r: r["v"])
    return {
        "tag": tag,
        "dist": s.get("trigger_distance_m"),
        "walker": s.get("walker_speed_mps") is not None,
        "v0": round(seq[0]["v"], 2),
        "coll": s.get("collision_pedestrian"),
        "vis": None if vis is None else round(vis, 2),
        "brake_on": None if onset is None else round(onset, 2),
        "duty3": round(len(br) * 0.05 / 3.0, 3),
        "pulses": len(rr),
        "longest": round(max(rr) * 0.05, 2) if rr else 0.0,
        "minv3": round(mv3["v"], 2),
        "minv3_t": round(mv3["tau"], 2),
        "minv5": round(mv5["v"], 2),
        "v_near": round(near["v"], 2) if near else None,
        "minD": round(near["dist"], 3) if near else None,
        "des0": round(seq[0]["des"], 2),
        "desmin3": round(min(r["des"] for r in w3), 2),
        "desmin3_t": round(min(w3, key=lambda r: r["des"])["tau"], 2),
        "clock_off": round(off, 3),
    }


if __name__ == "__main__":
    tags = [f[len("summary_"):-5] for f in sorted(os.listdir(A)) if f.startswith("summary_")]
    rows = [r for r in (metrics(t) for t in tags) if r]
    hdr = ["tag", "dist", "walker", "v0", "coll", "vis", "brake_on", "duty3", "pulses",
           "longest", "minv3", "minv3_t", "minv5", "v_near", "minD", "des0", "desmin3",
           "desmin3_t", "clock_off"]
    print("%-20s %4s %6s %6s %5s %5s %8s %6s %6s %7s %6s %7s %6s %6s %6s %6s %7s %8s" % tuple(
        h if h != "tag" else "tag" for h in hdr[:-1]))
    for r in rows:
        print("%-20s %4s %6s %6s %5s %5s %8s %6s %6s %7s %6s %7s %6s %6s %6s %6s %7s %8s" % (
            r["tag"], r["dist"], r["walker"], r["v0"], r["coll"], r["vis"], r["brake_on"],
            r["duty3"], r["pulses"], r["longest"], r["minv3"], r["minv3_t"], r["minv5"],
            r["v_near"], r["minD"], r["des0"], r["desmin3"], r["desmin3_t"]))
    with open(os.path.join(A, "window_metrics.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=hdr)
        w.writeheader()
        w.writerows(rows)
