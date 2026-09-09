#!/usr/bin/env bash
ALPINE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ALPINE_ROOT
export ROS_DOMAIN_ID="${ALPINE_ROS_DOMAIN_ID:-89}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="$ALPINE_ROOT/ros2/log/ros"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
if [[ -f "$ALPINE_ROOT/ros2/install/local_setup.bash" ]]; then
  source "$ALPINE_ROOT/ros2/install/local_setup.bash"
fi
