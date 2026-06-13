#!/usr/bin/env bash
# launch_tf_bridge.sh -- Build (if needed) and run the tf_bridge ROS2 node.
#
# Usage:
#   ./scripts/launch_tf_bridge.sh
#   ./scripts/launch_tf_bridge.sh --ros-args -p pi_ws_url:=ws://192.168.1.241:8000/ws
#   ./scripts/launch_tf_bridge.sh --ros-args -p confidence_threshold:=15.0

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"

source /opt/ros/humble/setup.bash

# Build if not yet installed or if source is newer than install
PKG_INSTALL="$ROS2_WS/install/tf_bridge"
PKG_SRC="$REPO_ROOT/ros2/tf_bridge"

if [ ! -d "$PKG_INSTALL" ] || [ "$PKG_SRC" -nt "$PKG_INSTALL" ]; then
    echo "[launch_tf_bridge] Building tf_bridge package..."
    mkdir -p "$ROS2_WS/src"
    if [ ! -L "$ROS2_WS/src/tf_bridge" ]; then
        ln -s "$PKG_SRC" "$ROS2_WS/src/tf_bridge"
    fi
    cd "$ROS2_WS"
    colcon build --packages-select tf_bridge --symlink-install
    cd - > /dev/null
fi

source "$ROS2_WS/install/setup.bash"

echo "[launch_tf_bridge] Starting tf_bridge node..."
exec ros2 run tf_bridge tf_bridge "$@"
