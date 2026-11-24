#!/bin/bash
# Install the GhostPedestrianV4 scenario into LEAD's ScenarioRunner.
set -euo pipefail
SRC=/root/autodl-tmp/carla-autodrive-validation/ghost_pedestrian_v4/scenarios/ghost_pedestrian_v4.py
DST_DIR=/root/autodl-tmp/third_party/lead/3rd_party/leaderboard/standard/scenario_runner/srunner/scenarios
DST="$DST_DIR/ghost_pedestrian_v4.py"
[ -f "$DST" ] && cp "$DST" "$DST.bak.$(date +%Y%m%d_%H%M%S)"
cp "$SRC" "$DST"
echo "installed $DST"
python3 -c "import ast,sys; ast.parse(open(sys.argv[1]).read()); print('syntax OK')" "$DST"
