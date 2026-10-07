#!/usr/bin/env bash
set -e
GHOST_PLUGIN_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$GHOST_PLUGIN_ROOT/src/gazebo_plugins" -B "$GHOST_PLUGIN_ROOT/runtime/build/plugins" -DCMAKE_BUILD_TYPE=Release > "$GHOST_PLUGIN_ROOT/runtime/logs/plugin_build.log" 2>&1
cmake --build "$GHOST_PLUGIN_ROOT/runtime/build/plugins" -j2 >> "$GHOST_PLUGIN_ROOT/runtime/logs/plugin_build.log" 2>&1
