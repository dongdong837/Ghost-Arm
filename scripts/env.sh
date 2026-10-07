#!/usr/bin/env bash
# Shared by all launchers; uses the checkout location, never the caller's cwd.
GHOST_SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43
export IGN_PARTITION=ghost_sim
export PYTHONPATH="$GHOST_SIM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$GHOST_SIM_ROOT/runtime/logs" "$GHOST_SIM_ROOT/runtime/reports" "$GHOST_SIM_ROOT/runtime/maps"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$GHOST_SIM_ROOT/runtime/build/plugins${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export LP_NUM_THREADS=2
