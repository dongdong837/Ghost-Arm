#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec 9> /tmp/ghost_teleop.lock
flock -n 9 || { echo '幽灵键盘控制已经运行。'; exit 1; }
exec python3 -m ghost_sim.control.keyboard
