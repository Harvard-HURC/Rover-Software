#!/usr/bin/env bash
# Start the rover simulation with its ROS 2 stack: pixi run sim [world] [--follow]
# world: a path to an .sdf, or a name in rover_sim/worlds (e.g. urc_autonomy).
# --follow: the GUI camera follows the rover (drive it with pixi run drive, or
# ros2 topic pub /cmd_vel). launch/sim.launch.py starts Gazebo's server and GUI
# as separate processes (macOS cannot run both in one), the rover's controllers
# and the sensor bridge; closing the GUI stops everything.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
# Our models and plugins, the real OGRE paths, the patched media: sim/gzenv.py,
# the one place every Gazebo user (station, tests, tools) takes them from.
eval "$(python "$here/gzenv.py")"

follow=0
world="$here/worlds/rover_test.sdf"
for arg in "$@"; do
  case "$arg" in
    --follow) follow=1 ;;
    *) world="$arg" ;;
  esac
done
if [[ ! -f "$world" && -f "$here/worlds/$world.sdf" ]]; then
  world="$here/worlds/$world.sdf"
fi
if [[ ! -f "$world" ]]; then  # a typo, or a world sim-worlds skipped because a raster is not here: say which
  why="$(python -c 'import sys; sys.path.insert(0, sys.argv[1]); import fetch_data
print(fetch_data.world_missing(sys.argv[2]) or "")' "$here/tools" "$world")"
  echo "run.sh: no world $world: ${why:-give an .sdf path or a name in rover_sim/worlds}" >&2
  exit 1
fi

gui_service() {  # name, request type, request: true if the GUI accepted it
  gz service -s "$1" --reqtype "$2" --reptype gz.msgs.Boolean --timeout 1000 --req "$3" 2>/dev/null \
    | grep -q "data: true"
}

# The GUI opens looking at the world origin; the URC worlds start the rover
# elsewhere, so point the camera at it once the GUI is up. With --follow the
# GUI's CameraTracking then keeps the camera 5 m behind and 2.5 m above it.
look_at_rover() {
  for _ in $(seq 120); do
    if gui_service /gui/move_to gz.msgs.StringMsg 'data: "rover"'; then
      if (( follow )); then  # /gui/follow resets the offset, so the offset goes second
        gui_service /gui/follow gz.msgs.StringMsg 'data: "rover"' || echo "run.sh: the GUI did not follow the rover" >&2
        gui_service /gui/follow/offset gz.msgs.Vector3d 'x: -5, y: 0, z: 2.5' || true
      fi
      return
    fi
    sleep 1
  done
}

look_at_rover &
helper=$!
trap 'kill "$helper" 2>/dev/null || true' EXIT
ros2 launch "$here/launch/sim.launch.py" "world:=$world" gui:=true
