# 前置 36 线激光雷达与 IMU 融合

仿真程序和 ROS 2 分开运行。仿真发送原始传感器数据，Linux 的 LIO-SAM 负责激光惯导里程计和建图。本交付包含桥接节点、融合启动文件和匹配参数；当前 Windows 上验证了真实数据传输，未运行 Linux/ROS 2/LIO-SAM，融合精度尚待 Linux 联调。

## 传感器接口

- `/points_raw`（默认关闭，`publish_raw_points: true` 开启）：sensor_msgs/PointCloud2，36 行 × 180 列，10 Hz，单圈 6480 个槽位，无回波点为 NaN。
- `/points_lio`：相同扫描的有效点，已去除 NaN，is_dense=true，供 LIO-SAM 使用。
- `/imu/data`：sensor_msgs/Imu，200 Hz，包含角速度、比力（含重力的静止读数约 9.81 m/s²）和理想姿态。
- 二维雷达已停用，不再发布 `/scan`。
- `/clock`：共用仿真时钟。IMU 与点云均使用此时钟。

点云字段：x/y/z/intensity 为 float32，ring 为 uint16（0–35），time 为 float32。每点 24 字节、小端。time=0 表示这一圈是同一时刻的理想快照；不伪造逐点扫描时间。查询分散到多个物理帧，但使用冻结的扫描起点姿态和静态场景，所以无需运动去畸变。

雷达直接安装在 PTZ 摄像头顶部，`base_link → lidar36_link` 的平移为 `[0.335, 0.0, 2.55]` 米（ROS：X 前、Y 左、Z 上）。虚拟 IMU 与雷达共享测量原点和方向，雷达到 IMU 的外参为零平移、单位旋转。IMU 比力计算包含该安装偏移产生的角加速度、向心加速度项。

`config/lio_sam_36.yaml` 已设置：sensor=velodyne（仅使用其 XYZIRT 消息格式）、N_SCAN=36、Horizon_SCAN=180、外参单位矩阵、重力9.81、use_sim_time=true。它不是某个真实雷达品牌的逐项物理模型。

## Linux 使用

推荐先在 Ubuntu 22.04 / ROS 2 Humble 上联调。先依据 LIO-SAM 官方 ros2 分支说明安装其依赖，尤其是 GTSAM 4.1：
https://github.com/TixiaoShan/LIO-SAM/tree/ros2

将本目录放入工作空间 src，再获取算法源码并编译。例如：

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/alpine_ws/src
# 将解压得到的 godot_ros2_bridge 整个目录复制到 ~/alpine_ws/src/
git clone --branch ros2 https://github.com/TixiaoShan/LIO-SAM.git ~/alpine_ws/src/LIO-SAM
cd ~/alpine_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

修改本包 `config/bridge.yaml` 中的 host（仿真电脑 IP）、port 和 token，与仿真目录的 sim_config.json 对应。随后只启动融合入口，无需再启动一个独立桥接进程：

```bash
ros2 launch godot_ros2_bridge fusion.launch.py bridge_config:=/绝对路径/bridge.yaml
```

这个入口同时运行网络桥接和 LIO-SAM 的四个处理节点。桥接仅发布传感器静态 TF，关闭仿真真值的 odom→base_link TF，避免与融合定位冲突。仿真真值仍可从 `/sim/odom` 和 `/sim/ground_truth/pose` 读取；它们不是融合结果。融合输出可在 `/lio_sam/mapping/odometry` 及 LIO-SAM 的建图话题查看。点云仅在 ROS 2 端使用，仿真世界不显示点云。

```bash
ros2 topic hz /imu/data
ros2 topic hz /points_lio
ros2 topic list
```

扫描密度可在仿真的 sim_config.json 调整 lidar_azimuth_samples（90–720）和 lidar_hz（1–10）。调整列数后，同步修改 lio_sam_36.yaml 的 Horizon_SCAN。默认预设照顾当前集成显卡；更密的点云可改善部分场景特征，但会增加开销。车辆复位/瞬移后重新启动融合入口，使算法重新初始化。

## 模型范围

雷达对地形、道路、岩石和结构进行射线测距；树冠和灌木使用简化探测体积。IMU 和雷达目前为理想传感器，没有标定过的硬件噪声、偏置漂移、多回波和反射率模型。参数已按真实上游接口适配，仍需在 Linux 上检验轨迹与优化参数。

LIO-SAM 参数来源及许可见 LIO_SAM_LICENSE.txt；本包本身不包含也不冒充已安装的 LIO-SAM 二进制。
