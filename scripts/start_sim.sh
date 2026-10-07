#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec 9> /tmp/ghost_sim.lock
flock -n 9 || { echo '幽灵仿真已经运行。'; exit 1; }
bash "$GHOST_SIM_ROOT/scripts/build_plugins.sh" || { cat "$GHOST_SIM_ROOT/runtime/logs/plugin_build.log" >&2; exit 1; }
exec ros2 launch "$GHOST_SIM_ROOT/launch/sim.launch.py" "$@"
