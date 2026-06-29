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
#
# ibvs_anchor_mode is ON by default below (tag0-based world frame for IBVS-only tracking).
# Remove it or set :=false to revert to tag2/3 pair-based tracking (world_ibvs / rat_chase).

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
# Pass ibvs_anchor_mode as a default; any extra --ros-args from $@ are appended.
# ROS2 supports multiple --ros-args segments on the same command line.
ros2 run tf_bridge tf_bridge --ros-args -p ibvs_anchor_mode:=true "$@"

kill $RSP_PID 2>/dev/null || true
