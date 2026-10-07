#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec timeout 10 ros2 service call /ghost/cancel_navigation std_srvs/srv/Trigger '{}'
