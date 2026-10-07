#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec python3 -m ghost_sim.ai.ghost_ai "$@"
