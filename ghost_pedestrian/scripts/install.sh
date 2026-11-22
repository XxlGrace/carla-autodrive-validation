#!/bin/bash
# Install the GhostPedestrian scenario into LEAD's standard ScenarioRunner.
# The evaluator auto-discovers every *.py in srunner/scenarios/, so no LEAD core
# code is modified. Keeps a timestamped backup of any previous copy.
set -euo pipefail

SRC=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian/scenarios/ghost_pedestrian.py
DST_DIR=/root/autodl-tmp/third_party/lead/3rd_party/leaderboard/standard/scenario_runner/srunner/scenarios
DST="$DST_DIR/ghost_pedestrian.py"

if [ -f "$DST" ]; then
  cp "$DST" "$DST.bak.$(date +%Y%m%d_%H%M%S)"
fi
cp "$SRC" "$DST"
echo "installed $DST"
python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read()); print('syntax OK')" "$DST"
