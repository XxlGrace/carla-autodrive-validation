#!/usr/bin/env python3
"""P6: small-scale active boundary search with a geometry-only baseline.

Both strategies get the same six CARLA queries. They start from the P4/P5
runs already on disk, fit a logistic collision model, select the candidate
whose predicted collision probability is closest to 0.5, run it, and refit.

  baseline  : features = [trigger_distance, walker_speed]          (geometry)
  informed  : features = [actual_trigger_distance, walker_speed,
                          ego_speed_at_release, brake_onset_after_release]
              (measured release timing + LEAD response; for unrun candidates the
               response features are k-NN-imputed from completed runs)

The comparison is the boundary-localisation error after the six queries.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

RUNS = "/root/autodl-tmp/ghost_runs"
REPO = "/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian"
RESULTS = os.path.join(REPO, "results")
RUNNER = os.path.join(REPO, "scripts", "run_batch.sh")
os.makedirs(RESULTS, exist_ok=True)


def load(name):
    p = os.path.join(RUNS, name, "summary.json")
    if not os.path.exists(p):
        return None
    try:
        s = json.load(open(p))
        return s if s.get("status") == "final" else None
    except Exception:
        return None


def collect():
    rows = []
    for name in sorted(os.listdir(RUNS)):
        if not name.startswith(("p4_", "p5_", "p6_")):
            continue
        s = load(name)
        if not s:
            continue
        sp = None
        d = s.get("trigger_distance_m")
        if name.startswith("p5_") or name.startswith("p6_"):
            parts = name.split("_")
            sp = float(parts[1][1:].rstrip("m").replace("p", "."))
            d = float(parts[2][1:].rstrip("m").replace("p", "."))
        else:
            sp = 1.5
        if d is None or sp is None:
            continue
        rows.append({
            "name": name,
            "d": float(d),
            "speed": float(sp),
            "actual": s.get("actual_trigger_distance_m"),
            "v0": s.get("ego_speed_at_release_mps"),
            "brake": s.get("brake_onset_after_release_s"),
            "y": 1 if s.get("collision_pedestrian") else 0,
        })
    return rows


def candidates():
    out = []
    for name in sorted(os.listdir(os.path.join(REPO, "routes"))):
        if name.startswith("p6_") and name.endswith(".xml"):
            run = name[:-4]
            parts = run.split("_")
            out.append({"name": run, "d": float(parts[2][1:].rstrip("m").replace("p", ".")),
                        "speed": float(parts[1][1:].rstrip("m").replace("p", "."))})
    return out


def fit_logistic(X, y, l2=0.1, iters=800, lr=0.05):
    if len(y) < 2 or len(set(y)) < 2:
        return np.zeros(X.shape[1]), 0.0
    X = np.asarray(X, dtype=float)
    mu, sd = X.mean(0), X.std(0)
    sd[sd < 1e-9] = 1.0
    Z = (X - mu) / sd
    w = np.zeros(Z.shape[1])
    b = 0.0
    y = np.asarray(y, dtype=float)
    for _ in range(iters):
        p = 1.0 / (1.0 + np.exp(-np.clip(Z @ w + b, -30, 30)))
        err = p - y
        gw = Z.T @ err / len(y) + l2 * w
        gb = err.mean()
        w -= lr * gw
        b -= lr * gb
    return w, b, (mu, sd)


def predict(model, X):
    w, b, (mu, sd) = model
    Z = (np.asarray(X, dtype=float) - mu) / sd
    return 1.0 / (1.0 + np.exp(-np.clip(Z @ w + b, -30, 30)))


def features(row, strategy, pool):
    d = row["d"]
    speed = row["speed"]
    if strategy == "baseline":
        return [d, speed]
    # k-NN imputation of the measured release/response features.
    actual = row.get("actual")
    v0 = row.get("v0")
    brake = row.get("brake")
    if actual is None or v0 is None or brake is None:
        near = sorted(pool, key=lambda r: (r["d"] - d) ** 2 + 4 * (r["speed"] - speed) ** 2)[:3]
        def avg(key):
            vals = [r[key] for r in near if r.get(key) is not None]
            return float(np.mean(vals)) if vals else 0.0
        actual = avg("actual")
        v0 = avg("v0")
        brake = avg("brake")
    return [actual, speed, v0, brake if brake is not None else 0.0]


def run_route(name):
    # Use the batch runner so every query gets a clean CARLA and an isolated
    # traffic-manager port (run_ghost_test.sh reuses port 8000 and can collide).
    lst = "/tmp/p6_one_route.txt"
    with open(lst, "w") as f:
        f.write(name + ".xml\n")
    log = "/tmp/p6_%s.out" % name
    with open(log, "w") as fh:
        subprocess.call(["bash", RUNNER, lst, "1", "420"], stdout=fh, stderr=fh)
    return load(name)


def boundary_error(pool):
    """Boundary = distance where predicted P(collision)=0.5 at speed 1.5."""
    model = fit_logistic([[r["d"], r["speed"]] for r in pool],
                         [r["y"] for r in pool])
    grid = np.arange(18, 29, 0.25)
    p = predict(model, [[d, 1.5] for d in grid])
    cross = [float(d) for d, pi in zip(grid, p) if abs(pi - 0.5) < 0.08]
    return (float(np.mean(cross)) if cross else None), p


def main():
    budget = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    all_rows = collect()
    cands = candidates()
    report = {"budget": budget, "strategies": {}}
    ref, _ = boundary_error(all_rows)
    report["reference_boundary_m"] = ref
    print("reference boundary (all data): %s" % ref)

    for strategy in ("baseline", "informed"):
        pool = list(all_rows)
        chosen = []
        for k in range(budget):
            X = [features(r, strategy, pool) for r in pool]
            model = fit_logistic(X, [r["y"] for r in pool])
            scored = []
            for c in cands:
                if any(r["name"] == c["name"] for r in pool):
                    continue
                p = float(predict(model, [features(c, strategy, pool)])[0])
                scored.append((abs(p - 0.5), -p * (1 - p), c, p))
            if not scored:
                break
            scored.sort(key=lambda t: (round(t[0], 4), t[1]))
            pick = scored[0][2]
            print("%s query %d: %s d=%.2f speed=%.1f p=%.3f" % (
                strategy, k + 1, pick["name"], pick["d"], pick["speed"],
                scored[0][3]))
            s = run_route(pick["name"])
            if s is None:
                print("  FAILED (no summary); skipping")
                cands = [c for c in cands if c["name"] != pick["name"]]
                continue
            row = {
                "name": pick["name"], "d": pick["d"], "speed": pick["speed"],
                "actual": s.get("actual_trigger_distance_m"),
                "v0": s.get("ego_speed_at_release_mps"),
                "brake": s.get("brake_onset_after_release_s"),
                "y": 1 if s.get("collision_pedestrian") else 0,
            }
            pool.append(row)
            chosen.append({"name": row["name"], "outcome": "collision" if row["y"] else "safe",
                           "p_predicted": round(scored[0][3], 3)})
        b, probs = boundary_error(pool)
        err = abs(b - ref) if (b is not None and ref is not None) else None
        print("%s final boundary=%s error=%s collisions_found=%d" % (
            strategy, b, err, sum(c["outcome"] == "collision" for c in chosen)))
        report["strategies"][strategy] = {
            "chosen": chosen, "boundary_m": b, "error_m": err,
            "collisions_found": sum(c["outcome"] == "collision" for c in chosen),
        }
    with open(os.path.join(RESULTS, "p6_active_search.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print("saved", os.path.join(RESULTS, "p6_active_search.json"))


if __name__ == "__main__":
    main()
