#!/bin/bash
# Reproduce the whole GhostPedestrianV4 result set.
#   1) geometry self-check      2) LEAD sweep + controls + repeats + oracle + speed
#   3) demo runs (3 cameras)    4) analysis, doc, videos
set -uo pipefail
REPO=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4
RUNS=/root/autodl-tmp/ghost_runs_v4
PORT=2000
TMO="${1:-420}"

source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/lead

bash /root/autodl-tmp/v4_set_map.sh Town05_Opt >/dev/null
python3 "$REPO/scripts/gen_routes_v4.py"
bash "$REPO/scripts/install_v4.sh"

# 1) geometry self-check (pure CARLA, no model)
bash /root/autodl-tmp/v4_start_carla.sh "$PORT" Low
python /root/autodl-tmp/v4_wait.py "$PORT" 120 >/dev/null
python "$REPO/scripts/verify_geometry_v4.py" "$PORT" | tee "$REPO/results/geometry_check.txt"
bash /root/autodl-tmp/v4_kill_carla.sh "$PORT" >/dev/null 2>&1

# 2) experiments
{
  for d in 8 12 16 20 24 26 28 30 34 40; do echo "v4_d${d}m.xml pred"; done
  echo "v4_baseline.xml"
  echo "v4_occluder_only.xml"
  echo "v4_no_occluder.xml pred"
  echo "v4_d20m_or_immediate.xml pred"
  echo "v4_d20m_or_visible.xml pred"
  for r in 1 2 3; do echo "v4_boundary20_r${r}.xml pred"; done
  for r in 1 2 3; do echo "v4_boundary24_r${r}.xml pred"; done
  for s in 1p0 2p0; do echo "v4_s${s}_d20m.xml pred"; done
} > /tmp/v4_sweep.txt
bash "$REPO/scripts/run_v4.sh" /tmp/v4_sweep.txt "$TMO"

# 3) demos
{
  echo "v4_demo_occluder_only.xml pred"
  echo "v4_demo_collision.xml pred"
  echo "v4_demo_safe.xml pred"
  echo "v4_demo_oracle.xml pred"
} > /tmp/v4_demos.txt
bash "$REPO/scripts/run_v4.sh" /tmp/v4_demos.txt "$TMO"

# 4) results, doc, videos
python3 "$REPO/scripts/analyze_v4.py" | tee "$RUNS/analysis.txt"
python3 "$REPO/scripts/gen_v4_doc.py"
python3 "$REPO/scripts/make_demos_v4.py" | tee "$RUNS/demos.txt"
echo "ALL DONE"
