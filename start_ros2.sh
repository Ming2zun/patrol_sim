#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
rviz=true
build=false
for arg in "$@"; do
  case "$arg" in
    --build) build=true ;;
    --no-rviz) rviz=false ;;
    *) echo '用法：./start_ros2.sh [--build] [--no-rviz]'; exit 2 ;;
  esac
done
if [[ "$build" == true || ! -f ros2/install/alpine_sim_slam/share/alpine_sim_slam/package.xml ]]; then
  ./scripts/build_ros2.sh
fi
source ros2/env.sh
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/alpine-ros2-${UID}-${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "本 ROS 域的巡检服务已经启动，请先停止原终端中的服务。"
  exit 1
fi
sim_pid=''
ros_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$ros_pid" ]] && kill -0 "$ros_pid" 2>/dev/null; then kill -TERM -- "-$ros_pid" 2>/dev/null || true; wait "$ros_pid" || true; fi
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then kill -TERM "$sim_pid"; wait "$sim_pid" || true; fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM
if /usr/bin/python3 - <<'PY'
import socket
try:
    with socket.create_connection(('127.0.0.1',9090),timeout=.3): pass
except OSError: raise SystemExit(1)
PY
then
  echo '连接已有仿真实例：127.0.0.1:9090'
else
  ./start_sim.sh 9>&- &
  sim_pid=$!
fi
echo "巡检 ROS 域：$ROS_DOMAIN_ID；RViz=$rviz；Ctrl+C 停止本脚本启动的进程。"
setsid ros2 launch godot_ros2_bridge mapping.launch.py rviz:="$rviz" 9>&- &
ros_pid=$!
wait "$ros_pid"
