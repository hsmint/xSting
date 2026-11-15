#!/usr/bin/env bash
set -e

# ROS_DOMAIN_ID는 유지하고 Lyrical 환경변수만 분리해 Isaac Sim 내장 ROS 2 브리지 구동
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE_PY="/home/pa03/Documents/FINAL/mk2_ws/src/xsting_pick_place/scripts/setup_isaac_scene.py"
if [ ! -f "$SCENE_PY" ]; then
  SCENE_PY="$SCRIPT_DIR/setup_isaac_scene.py"
fi

exec env \
  -u ROS_DISTRO \
  -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  -u LD_LIBRARY_PATH \
  /opt/IsaacSim/isaac-sim.sh \
  --/app/enableCrashreporter=false \
  --enable isaacsim.ros2.bridge \
  --exec "$SCENE_PY" \
  "$@"
