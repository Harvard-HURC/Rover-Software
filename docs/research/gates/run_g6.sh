#!/usr/bin/env bash
# Gate G6 in both worlds: the full config with the 1280 x 720 rover camera, the spec's fallback, the
# lens-flare check on the chase camera and the full config with today's DiffDrive rover. One server at a
# time; each run's server log lands next to its world copy in GATES_SCRATCH/g6.
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
for world in urc_delivery urc_equipment_servicing; do
  for variant in rgbd1280 fallback chase_flare rgbd1280_dd; do
    python "$here/g6.py" "$world" "$variant" 10 | grep -E "RESULT|Error|Traceback" || echo "g6 $world $variant: no result"
  done
done
