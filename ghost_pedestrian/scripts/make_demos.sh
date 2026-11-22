#!/bin/bash
# Build demo videos from the ego-attached camera frames saved by GhostPedestrian.
# These are headless CARLA renders of the scenario camera (not LEAD's input rig).
set -uo pipefail
RUNS=/root/autodl-tmp/ghost_runs
OUT=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/demos
TMP=/tmp/gpdemo
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
mkdir -p "$OUT" "$TMP"

make_one() {
  local run="$1" label="$2" outname="$3"
  local src="$RUNS/$run/frames"
  local dst="$TMP/$run"
  if [ ! -d "$src" ]; then echo "MISSING FRAMES $run"; return 1; fi
  rm -rf "$dst"; mkdir -p "$dst"
  local i=0
  for f in $(find "$src" -maxdepth 1 -name '*.png' | sort); do
    i=$((i+1))
    cp "$f" "$(printf '%s/%05d.png' "$dst" "$i")"
  done
  [ "$i" -gt 0 ] || { echo "EMPTY $run"; return 1; }
  ffmpeg -y -loglevel error -framerate 10 -i "$dst/%05d.png" \
    -vf "drawtext=fontfile=$FONT:text='$label':x=12:y=12:fontsize=20:fontcolor=white:box=1:boxcolor=black@0.55" \
    -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p \
    "$OUT/$outname.mp4"
  echo "made $OUT/$outname.mp4 from $i frames ($run)"
}

make_one occluder_only "Occluder only - no pedestrian" demo_occluder_only
make_one gp_25m "LEAD - 25 m trigger - SAFE" demo_safe_25m
make_one gp_24m "LEAD - 24 m trigger - COLLISION" demo_collision_24m
make_one gp_24m_or_visible "Oracle brake at first visible - SAFE" demo_oracle_brake

# 2x2 comparison: equal-length clips, scaled and labelled by the embedded text.
for f in demo_occluder_only demo_safe_25m demo_collision_24m demo_oracle_brake; do
  ffmpeg -y -loglevel error -i "$OUT/$f.mp4" -t 6 -vf scale=640:360 \
    -an -c:v libx264 -preset veryfast "$TMP/${f}_trim.mp4"
done
ffmpeg -y -loglevel error \
  -i "$TMP/demo_occluder_only_trim.mp4" -i "$TMP/demo_safe_25m_trim.mp4" \
  -i "$TMP/demo_collision_24m_trim.mp4" -i "$TMP/demo_oracle_brake_trim.mp4" \
  -filter_complex "[0:v][1:v]hstack=inputs=2[top];[2:v][3:v]hstack=inputs=2[bot];[top][bot]vstack=inputs=2[grid]" \
  -map "[grid]" -an -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p \
  "$OUT/demo_comparison.mp4"
echo "made $OUT/demo_comparison.mp4"
