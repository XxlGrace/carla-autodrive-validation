#!/bin/bash
# Stop CARLA started by start_carla.sh
set -uo pipefail

LOG_DIR=/root/autodl-tmp/carla-logs
PID_FILE="$LOG_DIR/carla.pid"

if [ -f "$PID_FILE" ]; then
  pid=$(cat "$PID_FILE")
  if kill -0 "$pid" 2>/dev/null; then
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
    sleep 3
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
fi

# Fallback: any leftover UE4 process for this CARLA package
pkill -f 'CARLA_0.9.16/CarlaUE4' 2>/dev/null || true
echo "CARLA stopped."
