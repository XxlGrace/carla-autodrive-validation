#!/bin/bash
# Run GhostPedestrianV4 routes sequentially with clean CARLA restarts.
# The CARLA map (Town05) is configured at startup via DefaultEngine.ini.
# Usage: run_v4.sh <route-list> [timeout]   ("pred" enables GP_PRED_LOG)
set -uo pipefail
LIST="${1:?usage: run_v4.sh <route-list> [timeout]}"
TMO="${2:-420}"
REPO=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4
RUNS=/root/autodl-tmp/ghost_runs_v4
LOGROOT=/root/autodl-tmp/carla-logs/v4
QUALITY="${CARLA_QUALITY:-Low}"
PORT=2000
TMPORT=9100
mkdir -p "$RUNS" "$LOGROOT"

source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/lead
export PYTHONUNBUFFERED=1 HF_HUB_OFFLINE=1 WANDB_MODE=offline
export NUMBA_NUM_THREADS=16 OMP_NUM_THREADS=16

while read -r route extra; do
  [ -z "$route" ] && continue
  name="${route%.xml}"
  echo "START $name quality=$QUALITY $(date +%H:%M:%S)"
  bash /root/autodl-tmp/v4_kill_carla.sh "$PORT" >/dev/null 2>&1
  bash /root/autodl-tmp/v4_start_carla.sh "$PORT" "$QUALITY"
  if ! python /root/autodl-tmp/v4_wait.py "$PORT" 120 >/dev/null 2>&1; then
    echo "  CARLA_NOT_READY"
    : > "$RUNS/$name.failed"
    continue
  fi
  if [ "${extra:-}" = "pred" ]; then
    mkdir -p "$RUNS/$name"
    : > "$RUNS/$name/pred.jsonl"
    export GP_PRED_LOG="$RUNS/$name/pred.jsonl"
  else
    unset GP_PRED_LOG
  fi
  cd /root/autodl-tmp/third_party/lead
  timeout "$TMO" python -m lead --checkpoint /root/autodl-tmp/third_party/lead/checkpoints/transfuser \
    --routes "$REPO/routes/$route" --port "$PORT" \
    --traffic-manager-port "$TMPORT" --timeout 300 \
    > "$RUNS/$name.eval.log" 2>&1
  rc=$?
  echo "$rc" > "$RUNS/$name.rc"
  echo "DONE $name rc=$rc $(date +%H:%M:%S)"
  sleep 2
done < "$LIST"
bash /root/autodl-tmp/v4_kill_carla.sh "$PORT" >/dev/null 2>&1
echo "V4 BATCH COMPLETE $(date +%H:%M:%S)"
