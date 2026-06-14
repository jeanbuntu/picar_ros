#!/usr/bin/env bash
# launch_tf_bridge.sh -- Build (if needed) and run tf_bridge + robot_state_publisher.
#
# picar_ros IS the colcon workspace. Run colcon build from repo root,
# then source install/setup.bash and launch.
#
# Usage:
#   ./scripts/launch_tf_bridge.sh
#   ./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws
#   ./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://localhost:8000/ws
#   ./scripts/launch_tf_bridge.sh --ros-args -p confidence_threshold:=15.0

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash

echo "[launch_tf_bridge] Building packages in workspace: $REPO_ROOT"
cd "$REPO_ROOT"
colcon build --packages-select tf_bridge picar_description --symlink-install 2>&1 | tail -5

source "$REPO_ROOT/install/setup.bash"

echo "[launch_tf_bridge] Starting robot_state_publisher..."
ros2 launch picar_description tracking_viz.launch.py &
RSP_PID=$!

echo "[launch_tf_bridge] Starting tf_bridge node..."
ros2 run tf_bridge tf_bridge "$@"

kill $RSP_PID 2>/dev/null || true
