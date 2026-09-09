#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source ros2/env.sh
cd ros2
colcon build --symlink-install --parallel-workers 2 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
printf 'ROS 编译完成。运行 ./start_ros2.sh 启动仿真、建图和 RViz。\n'
