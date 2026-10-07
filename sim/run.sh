#!/usr/bin/env bash
# Start the rover simulation: pixi run sim [world] [--follow]
# world: a path to an .sdf, or a name in sim/worlds (e.g. urc_autonomy).
# --follow: the GUI camera follows the rover (drive it with pixi run drive).
# macOS cannot run Gazebo's server and GUI in one process, so the server runs
# in the background and the GUI in the foreground; closing the GUI stops both.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$here/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$here/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
# The conda gz-rendering has a space-padded OGRE plugin path baked in; without
# this, cameras (and on some machines the GUI) cannot render.
prefix="${CONDA_PREFIX:-$here/../.pixi/envs/default}"
export OGRE2_RESOURCE_PATH="${OGRE2_RESOURCE_PATH:-$prefix/lib/OGRE-Next}"
export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:-$prefix/lib/OGRE}"

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

if [[ "$(uname)" == "Darwin" ]]; then
  gz sim -s -r "$world" &
  server=$!
  look_at_rover &
  helper=$!
  trap 'kill "$server" "$helper" 2>/dev/null || true' EXIT
  gz sim -g
else
  look_at_rover &
  trap 'kill $! 2>/dev/null || true' EXIT
  gz sim -r "$world"
fi
