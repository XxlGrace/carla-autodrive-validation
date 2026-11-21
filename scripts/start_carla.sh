#!/bin/bash
# Start CARLA 0.9.16 headless (Vulkan + RenderOffScreen) as non-root user.
set -euo pipefail

CARLA_DIR=/root/autodl-tmp/CARLA_0.9.16
LOG_DIR=/root/autodl-tmp/carla-logs
RUNNER=carla-runner
QUALITY=${CARLA_QUALITY:-Low}
RPC_PORT=${CARLA_RPC_PORT:-2000}

mkdir -p "$LOG_DIR" "$CARLA_DIR/CarlaUE4/Saved"

# Only use custom ICD when it actually exists
ICD=/etc/vulkan/icd.d/nvidia_icd.json
if [ -f "$ICD" ]; then
  export VK_ICD_FILENAMES="$ICD"
fi

LOG_FILE="$LOG_DIR/server.log"
echo "$LOG_FILE" > "$LOG_DIR/current_log.txt"

RUN_CMD="cd '$CARLA_DIR' && exec ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=$QUALITY -carla-rpc-port=$RPC_PORT -world-port=$RPC_PORT -graphicsadapter=0 -resx=800 -resy=600"
if [ -n "${VK_ICD_FILENAMES:-}" ]; then
  runuser -u "$RUNNER" -- env VK_ICD_FILENAMES="$VK_ICD_FILENAMES" __NV_PRIME_RENDER_OFFLOAD=1 CARLA_QUALITY="$QUALITY" \
    bash -c "$RUN_CMD" > "$LOG_FILE" 2>&1 &
else
  runuser -u "$RUNNER" -- env CARLA_QUALITY="$QUALITY" \
    bash -c "$RUN_CMD" > "$LOG_FILE" 2>&1 &
fi

echo $! > "$LOG_DIR/carla.pid"
echo "CARLA starting (PID $(cat "$LOG_DIR/carla.pid")), quality=$QUALITY, rpc=$RPC_PORT"
echo "Log: $LOG_FILE"
