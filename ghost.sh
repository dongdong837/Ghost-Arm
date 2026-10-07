#!/usr/bin/env bash
set -e
GHOST_ENTRY_ROOT="$(cd "$(dirname "$0")" && pwd)"
command_name="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$command_name" in
  start) exec "$GHOST_ENTRY_ROOT/scripts/start_sim.sh" "$@" ;;
  view) exec "$GHOST_ENTRY_ROOT/scripts/view_ghost.sh" "$@" ;;
  teleop) exec "$GHOST_ENTRY_ROOT/scripts/teleop.sh" "$@" ;;
  goto) exec "$GHOST_ENTRY_ROOT/scripts/navigate.sh" "$@" ;;
  ai) exec "$GHOST_ENTRY_ROOT/scripts/ghost_ai.sh" "$@" ;;
  stop) exec "$GHOST_ENTRY_ROOT/scripts/cancel_navigation.sh" "$@" ;;
  package) exec "$GHOST_ENTRY_ROOT/scripts/package.sh" "$@" ;;
  grasp|camera|ik|fk|arm|test|check-flight|check-navigation|build-model|map)
    source "$GHOST_ENTRY_ROOT/scripts/env.sh"
    case "$command_name" in
      grasp) exec python3 -m ghost_sim.control.grasp_demo "$@" ;;
      camera) exec python3 -m ghost_sim.perception.depth_camera "$@" ;;
      ik) exec python3 -m ghost_sim.kinematics.ik_cli "$@" ;;
      fk) exec python3 -m ghost_sim.kinematics.fk_cli "$@" ;;
      arm) exec python3 -m ghost_sim.control.arm_pose "$@" ;;
      test) exec python3 -m unittest discover -s "$GHOST_SIM_ROOT/tests/unit" -p 'test_*.py' -v ;;
      check-flight) exec python3 "$GHOST_SIM_ROOT/tests/integration/check_flight.py" "$@" ;;
      check-navigation) exec python3 "$GHOST_SIM_ROOT/tests/integration/check_navigation3d.py" "$@" ;;
      build-model) exec python3 -m ghost_sim.simulation.create_scene "$@" ;;
      map) exec python3 -m ghost_sim.mapping.voxel_map "$@" ;;
    esac ;;
  help|-h|--help)
    echo 'Usage: ./ghost.sh {start|view|teleop|grasp [pick|release]|camera [--pixel U V]|ik X Y Z [--pitch P] [--live|--execute]|fk {--joints Q1 Q2 Q3 Q4|--live}|arm {hang|ready|close}|goto X Y Z [--yaw R]|ai "任务"|stop|map|build-model|test|check-flight|check-navigation|package}' ;;
  *) echo "未知命令：$command_name。运行 ./ghost.sh help 查看用法。" >&2; exit 2 ;;
esac
