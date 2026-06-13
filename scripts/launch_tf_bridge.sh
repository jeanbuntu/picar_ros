#!/usr/bin/env bash
# launch_tf_bridge.sh -- Build (if needed) and run the tf_bridge ROS2 node.
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

echo "[launch_tf_bridge] Building tf_bridge in workspace: $REPO_ROOT"
cd "$REPO_ROOT"
colcon build --packages-select tf_bridge --symlink-install 2>&1 | tail -5

source "$REPO_ROOT/install/setup.bash"

echo "[launch_tf_bridge] Starting tf_bridge node..."
exec ros2 run tf_bridge tf_bridge "$@"
