# v008 阿克曼底盘

/cmd_vel 保持 Twist 接口，linear.x 是后轴中心纵向速度，angular.z 是期望偏航角速度；机械转角与高速限制会约束实际转弯。停车时车身不会原地旋转。导航应使用汽车式/阿克曼运动约束。

# v007 更新：摄像头顶部 36 线三维雷达 + 200 Hz IMU

新融合入口与完整说明见 `FUSION_README.md`。当前 IMU 位于前置雷达共同原点 `[0.335, 0, 2.55]` 米；下文早期版本中 IMU 在底盘原点的说明已被替代。

# Linux 上的 ROS 2 桥接包

此目录可以单独拷贝到 Linux；不需要复制 Godot、Blender 或三维资源。
面向 ROS 2 Jazzy（Ubuntu 24.04）编写，使用 rclpy 和标准消息；其他 ROS 2 版本需在目标机器验证。

## 1. 构建

先安装并 source 你的 ROS 2 环境，然后：
```bash
mkdir -p ~/alpine_ws/src
# 将整个 godot_ros2_bridge 文件夹放入 ~/alpine_ws/src/
cd ~/alpine_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select godot_ros2_bridge
source install/setup.bash
```

需要的标准 ROS 包已写入 package.xml。若系统没有 rosdep / colcon，先按你的 ROS 2 发行版安装它们。

## 2. 修改地址与启动

config/bridge.yaml 已带有与本次仿真发布包匹配的 token。
修改 host 为运行仿真程序的电脑 IP，或直接从 launch 参数传入：
```bash
ros2 launch godot_ros2_bridge bridge.launch.py host:=192.168.1.100
```

上面的 IP 是示例，必须替换成实际仿真电脑地址。
启动 Godot 程序；同机使用 127.0.0.1，不同机器通过 TCP 9090 连接。
两端端口与 token 必须一致。连接失败时先确认 IP、端口可达和防火墙规则。
桥接会自动重连，重连后不会恢复旧的速度指令。

## 3. 让 ROS 2 接管车辆

在仿真界面点击“ROS 2 远程控制”，或者：
```bash
ros2 service call /sim/set_remote std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

Ctrl+C 停止发布后，桥接超时归零，车辆制动停车。
要左转可将 angular.z 改为 0.3；单位为 rad/s。
启用 cmd_vel_stamped: true 时订阅类型改为 TwistStamped，匹配需要带时间戳指令的导航配置。

## 4. 检查数据

```bash
ros2 topic echo /sim/connected
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /camera/image_raw/compressed
ros2 run tf2_ros tf2_echo odom base_link
```

相机只有在仿真程序使用图形模式运行时输出。
图像类型为 sensor_msgs/CompressedImage，JPEG 320×180；/camera/camera_info 提供针孔内参。
RViz 中使用 odom 作为 Fixed Frame，添加 LaserScan /scan、TF、Odometry。
SLAM/Nav2 等算法节点应设置 use_sim_time: true。
桥接网络泵使用单调墙钟，即使仿真暂停也能继续处理连接和服务。

## 5. 服务

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /sim/pause std_srvs/srv/SetBool "{data: false}"
ros2 service call /sim/estop std_srvs/srv/SetBool "{data: true}"
ros2 service call /sim/estop std_srvs/srv/SetBool "{data: false}"
ros2 service call /sim/set_remote std_srvs/srv/SetBool "{data: false}"
```

解除急停后需要重新发布速度命令。复位车辆不会回拨 /clock。
服务返回 success 需要仿真端返回确认；断线时返回失败。

## 6. 后续修改位置

- config/bridge.yaml：地址、端口、token、话题名、坐标系名。
- godot_ros2_bridge/bridge_node.py：ROS 2 消息映射、TF、发布和服务。
- godot_ros2_bridge/transport.py：独立 TCP 协议、重连、队列和状态校验。
- launch/bridge.launch.py：ROS 2 启动入口。

位姿、速度和 IMU 已在 Godot 中转换为 ROS 坐标，桥接不要再次转换。
/odom 与 /ground_truth/pose 当前为理想真值；IMU 没有噪声模型。
base_link 原点位于车体底部逻辑坐标，IMU 与 base 原点重合；
laser_link 在上方 1.8 m，camera_optical_frame 在前方 0.7 m、上方 1.5 m。

## 验证状态

已经在 Windows 上使用真实 Godot 发布程序验证 TCP 协议、坐标方向、移动、传感器、
认证、暂停、急停、复位、超时和重连。
ROS 2 包已做 Python 语法和独立传输协议测试；本次没有 Linux/ROS 2 运行环境，
因此 rclpy 节点和 Linux 可执行程序仍需在目标 Linux 上实际验证。

纯 Python 协议测试（不依赖 ROS 2）：
```bash
cd godot_ros2_bridge
python3 -m unittest discover -s test -v
```


## 本机接口整理（2026-09-09）

- 不再发布二维 `/scan` 和 `laser_link` 静态变换。Godot 源码已删除二维扫描；现有 Linux 导出包通过 `laser_samples: 0` 停止二维射线查询，无需重新导出即可生效，重启仿真后生效。
- 默认只发布 `/points_lio`（有效点，保留 x/y/z/intensity/ring/time）。需要原始有组织点云时设置 `publish_raw_points: true`，才额外发布 `/points_raw`。
- ROS 2 Humble 使用 `from rclpy.clock import ClockType`。
- IMU 目标 200 Hz、点云目标 10 Hz，使用仿真时间。接收 IMU 时建议 best-effort、KeepLast 深度至少 50，避免批量到达时小队列丢消息；测量时同时观察仿真时间与现实时间之比。
