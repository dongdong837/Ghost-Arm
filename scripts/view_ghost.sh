#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec rviz2 -d "$GHOST_SIM_ROOT/config/ghost_camera.rviz" --ros-args -p use_sim_time:=true
