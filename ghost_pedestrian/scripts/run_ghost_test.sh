#!/bin/bash
# Run one ghost-pedestrian route through the LEAD closed-loop evaluator.
# Usage: run_ghost_test.sh <route_name.xml> [timeout_seconds]
#   e.g. run_ghost_test.sh gp_25m.xml 600
set -euo pipefail

ROUTE_NAME="${1:?usage: run_ghost_test.sh <route_name.xml> [timeout]}"
TMO="${2:-600}"
REPO=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian
ROUTE="$REPO/routes/$ROUTE_NAME"
RUNS=/root/autodl-tmp/ghost_runs
mkdir -p "$RUNS"
NAME="${ROUTE_NAME%.xml}"
LOG="$RUNS/$NAME.eval.log"

echo "=== clean CARLA restart ==="
/root/autodl-tmp/restart_carla_clean.sh >/dev/null 2>&1 || {
  echo "CARLA restart failed"; exit 1;
}

echo "=== eval $ROUTE_NAME (timeout ${TMO}s) -> $LOG ==="
source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/lead
cd /root/autodl-tmp/third_party/lead
export PYTHONUNBUFFERED=1 HF_HUB_OFFLINE=1 WANDB_MODE=offline
export NUMBA_NUM_THREADS=24 OMP_NUM_THREADS=24
export http_proxy=http://127.0.0.1:1080 https_proxy=http://127.0.0.1:1080 \
       no_proxy=127.0.0.1,localhost,::1

set +e
timeout "$TMO" python -m lead --checkpoint checkpoints/transfuser \
  --routes "$ROUTE" --port 2000 --timeout 300 > "$LOG" 2>&1
RC=$?
set -e

echo "[GP] eval rc=$RC"
echo "[GP] scenario markers:"; grep -E "\[GP\]|GhostPedestrian|EVALUATION|Results of RouteScenario" "$LOG" | tail -20 || true
echo "[GP] summary:"; cat "$RUNS/$NAME/summary.json" 2>/dev/null || echo "(no summary.json)"
echo "$RC" > "$RUNS/$NAME.rc"
exit 0
