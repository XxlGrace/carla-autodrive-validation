#!/bin/bash
# Remaining v3 experiment stages, run strictly sequentially (one CARLA at a time).
set -uo pipefail
REPO=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian
BATCH="$REPO/scripts/run_batch.sh"

echo "======== P3.5: LEAD internal timeline (paired runs, hook enabled) ========"
cat > /tmp/p35list.txt <<'EOF'
p35_lead_24m.xml pred
p35_or_24m.xml pred
EOF
bash "$BATCH" /tmp/p35list.txt 1 420

echo "======== P5: walker speed x distance grid ========"
: > /tmp/p5list.txt
for s in 1p0 1p5 2p0; do
  for d in 20m 22m 24m 26m; do
    echo "p5_s${s}_d${d}.xml" >> /tmp/p5list.txt
  done
done
bash "$BATCH" /tmp/p5list.txt 1 420

echo "======== P6: active boundary search ========"
python3 "$REPO/scripts/p6_active_search.py" 6

echo "======== V3 EXPERIMENTS COMPLETE ========"
