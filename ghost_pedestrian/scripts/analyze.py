import json, os, re, glob
RUNS="/root/autodl-tmp/ghost_runs"
NAMES=["baseline","occluder_only","gp_25m","gp_15m","gp_9m"]
out=[]
for name in NAMES:
    d=os.path.join(RUNS,name)
    sp=os.path.join(d,"summary.json")
    s=json.load(open(sp)) if os.path.exists(sp) else {}
    tp=os.path.join(d,"trace.jsonl")
    recs=[json.loads(l) for l in open(tp)] if os.path.exists(tp) else []
    rel=[r for r in recs if r.get("released")]
    ttc=[r["ttc"] for r in recs if r.get("ttc") is not None]
    spd=[r["ego_speed"] for r in recs if r.get("ego_speed") is not None]
    lex=[r["ego_x"] for r in recs if r.get("ego_x") is not None]
    wx=[r["walker_x"] for r in recs if r.get("walker_x") is not None]
    crossed = bool(wx and min(wx) <= min(lex)+0.9 and max(wx) >= max(lex)-0.9)
    # leaderboard criteria
    log=os.path.join(RUNS,name+".eval.log")
    crit={}
    if os.path.exists(log):
        txt=re.sub(r"\x1b\[[0-9;]*[mK]","",open(log,errors="ignore").read())
        for key in ["RouteCompletionTest","CollisionTest","MinSpeedTest","AgentBlockedTest"]:
            m=re.search(key+r"\s*│\s*(SUCCESS|FAILURE)",txt)
            if m: crit[key]=m.group(1)
    out.append({
        "run":name,
        "trigger_m": {"gp_25m":25,"gp_15m":15,"gp_9m":9}.get(name,None),
        "status":s.get("status"),
        "collision_ped":s.get("collision_pedestrian",False),
        "min_dist_m":s.get("min_distance_m"),
        "release_s":None if s.get("release_time_s") is None else round(s["release_time_s"],2),
        "brake_after_rel_s":s.get("brake_onset_after_release_s"),
        "ego_speed_at_release":rel[0]["ego_speed"] if rel else None,
        "ttc_at_release_s": (round(({"gp_25m":25,"gp_15m":15,"gp_9m":9}[name])/rel[0]["ego_speed"],2) if rel and name in ("gp_25m","gp_15m","gp_9m") and rel[0]["ego_speed"]>0 else None),
        "min_ttc_s":round(min([t for t in ttc if t>0.5]),3) if ttc else None,
        "occ_frac":s.get("occluded_fraction_before_release"),
        "frames":s.get("frames_saved"),
        "walker_crossed_ego_path":crossed,
        "criteria":crit,
        "outcome": ("COLLISION" if s.get("collision_pedestrian") else
                    ("SAFE_AVOIDANCE" if s.get("brake_onset_after_release_s") is not None
                     and rel else "CONTROL")),
    })
json.dump(out,open(os.path.join(os.path.dirname(__file__),"..","results","summary.json"),"w"),indent=2)
print("%-14s %5s %8s %9s %8s %8s %8s %7s %5s %s"%("run","trig","collision","minDist","brakeRel","spdRel","ttcRel","occ","cross","outcome"))
for r in out:
    print("%-14s %5s %8s %9s %8s %8s %8s %7s %5s %s"%(r["run"],r["trigger_m"],r["collision_ped"],r["min_dist_m"],r["brake_after_rel_s"],r["ego_speed_at_release"],r["ttc_at_release_s"],r["occ_frac"],r["walker_crossed_ego_path"],r["outcome"]))
