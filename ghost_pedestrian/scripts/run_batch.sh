#!/bin/bash
# Run a list of ghost-pedestrian routes with several independent CARLA workers.
# Usage: run_batch.sh <route-list> <workers> [timeout]
#   route-list: one route XML name per line; append " pred" to enable GP_PRED_LOG
# Each worker owns one CARLA RPC port and its own log, and restarts CARLA before
# every route so runs stay as independent as the sequential runner.
set -uo pipefail

LIST="${1:?usage: run_batch.sh <route-list> <workers> [timeout]}"
WORKERS="${2:-2}"
TMO="${3:-420}"
REPO=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian
RUNS=/root/autodl-tmp/ghost_runs
CARLA_DIR=/root/autodl-tmp/CARLA_0.9.16
LOGROOT=/root/autodl-tmp/carla-logs/batch
mkdir -p "$RUNS" "$LOGROOT" /tmp/gpbatch

source /root/miniconda3/etc/profile.d/conda.sh
conda activate /root/autodl-tmp/envs/lead
export PYTHONUNBUFFERED=1 HF_HUB_OFFLINE=1 WANDB_MODE=offline
export NUMBA_NUM_THREADS=16 OMP_NUM_THREADS=16
export http_proxy=http://127.0.0.1:1080 https_proxy=http://127.0.0.1:1080 \
       no_proxy=127.0.0.1,localhost,::1

# Split route list round-robin into per-worker files.
rm -f /tmp/gpbatch/worker_*.lst
awk -v n="$WORKERS" '{print > ("/tmp/gpbatch/worker_" ((NR-1)%n) ".lst")}' "$LIST"

start_carla() {
  local port="$1" logdir="$2"
  mkdir -p "$logdir" "$CARLA_DIR/CarlaUE4/Saved"
  local icd=/etc/vulkan/icd.d/nvidia_icd.json
  local cmd="cd '$CARLA_DIR' && exec ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low -carla-rpc-port=$port -world-port=$port -graphicsadapter=0 -resx=800 -resy=600"
  if [ -f "$icd" ]; then
    runuser -u carla-runner -- env VK_ICD_FILENAMES="$icd" __NV_PRIME_RENDER_OFFLOAD=1 CARLA_QUALITY=Low \
      bash -c "$cmd" > "$logdir/server.log" 2>&1 &
  else
    runuser -u carla-runner -- env CARLA_QUALITY=Low bash -c "$cmd" > "$logdir/server.log" 2>&1 &
  fi
  local ready=0
  for i in $(seq 1 60); do
    local r
    r=$(python - <<PY 2>/dev/null
import carla
try:
    c=carla.Client("127.0.0.1", $port); c.set_timeout(2)
    print("OK" if c.get_world() is not None else "NO")
except Exception:
    print("NO")
PY
)
    if [ "$r" = "OK" ]; then ready=1; break; fi
    sleep 2
  done
  if [ "$ready" -ne 1 ]; then
    echo "WORKER port=$port CARLA_NOT_READY"
    tail -10 "$logdir/server.log"
    return 1
  fi
  return 0
}

worker() {
  local wid="$1"
  local port=$((2000 + 2*wid))
  local logdir="$LOGROOT/w$wid"
  local lst="/tmp/gpbatch/worker_${wid}.lst"
  [ -s "$lst" ] || return 0
  local pidfile="$logdir/carla.pid"
  while read -r route extra; do
    [ -z "$route" ] && continue
    local name="${route%.xml}"
    echo "WORKER$wid START $name port=$port $(date +%H:%M:%S)"
    pkill -9 -f "carla-rpc-port=$port" 2>/dev/null || true
    sleep 2
    if ! start_carla "$port" "$logdir"; then
      echo "$name" > "/tmp/gpbatch/FAILED_$name"
      continue
    fi
    local predlog=""
    if [ "${extra:-}" = "pred" ]; then
      predlog="$RUNS/$name/pred.jsonl"
      mkdir -p "$(dirname "$predlog")"
      : > "$predlog"
      export GP_PRED_LOG="$predlog"
    else
      unset GP_PRED_LOG
    fi
    local tmport=$((9100 + wid))
    timeout "$TMO" python -m lead --checkpoint checkpoints/transfuser \
      --routes "$REPO/routes/$route" --port "$port" \
      --traffic-manager-port "$tmport" --timeout 300 \
      > "$RUNS/$name.eval.log" 2>&1
    local rc=$?
    echo "$rc" > "$RUNS/$name.rc"
    sleep 2
    echo "WORKER$wid DONE $name rc=$rc $(date +%H:%M:%S)"
  done < "$lst"
  pkill -9 -f "carla-rpc-port=$port" 2>/dev/null || true
}

pids=""
for w in $(seq 0 $((WORKERS-1))); do
  worker "$w" &
  pids="$pids $!"
done
for p in $pids; do wait "$p"; done
echo "BATCH COMPLETE $(date +%H:%M:%S)"
