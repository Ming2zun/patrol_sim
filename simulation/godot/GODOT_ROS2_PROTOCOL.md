# Godot / ROS 2 进程间协议 v1

传输：TCP，默认 9090。UTF-8 JSON，每条以 LF 结束，允许 TCP 分包、合包。
单位：米、秒、弧度。数据在仿真端已经转换为 ROS 坐标；ROS 桥接不得再次变换。
只允许一个客户端。连接、控制和仿真时间彼此独立。

## 握手

客户端首先发送：
```json
{"type":"hello","version":1,"token":"从 sim_config.json 读取"}
```
服务端返回 hello/version/capabilities。认证失败或超时会断开连接。
协议流不会执行远端 Python、GDScript 或任意代码。

## 控制

```json
{"type":"set_mode","mode":"ros","request_id":"1"}
{"type":"cmd_vel","linear_x":1.0,"angular_z":0.2,"seq":1}
{"type":"pause","enabled":true,"request_id":"2"}
{"type":"estop","enabled":true,"request_id":"3"}
{"type":"reset","request_id":"4"}
```

mode 支持 manual / route / ros。
linear_x、angular_z 必须是有限数值，按 sim_config.json 的上限裁剪。
seq 在当前有效控制序列中严格递增；模式切换/复位/断线清空序列和目标速度。
有效 cmd_vel 仅在 ros 模式、非暂停、非急停状态接受；超时基于单调墙钟而非仿真时钟。
服务端返回 ack，包含 request_id、operation、ok、mode、paused、estop。
ping 返回 pong，可检查链路活性。

## 状态帧

state 包含：
- version、seq、epoch、sim_time、mode、paused、estop、watchdog_stopped；
- pose.position [x,y,z]、pose.orientation [qx,qy,qz,qw]，均相对 odom；
- twist.linear / angular：base_link 局部速度；
- imu.orientation / angular_velocity / linear_acceleration：姿态、局部角速度、局部比力；
- laser.stamp、angle_min、angle_increment、range_min、range_max、scan_time、ranges；
- applied_command：当前生效目标。

laser 未命中使用 JSON null；ROS 端转换为 +inf。扫描为瞬时快照，time_increment=0。
暂停仍可回传状态，时间不增长；reset 只复位车辆、增加 epoch，不回拨时钟。
参考真实状态示例见 ROS 包 test/fixture_state.json。

## 相机帧

image 包含 version、sim_time、width、height、format="jpeg"、fx、fy、cx、cy、data。
data 是 JPEG 的 Base64 字符串。默认 320×180、目标 5 Hz，仅图形运行模式提供。
ROS 桥接转换为 CompressedImage 与 CameraInfo；不需要 OpenCV。
相机在 base_link 前方 0.7 m、上方 1.5 m，光学坐标系为 X 右/Y 下/Z 前。
二维雷达位于 base_link 上方 1.8 m。

## 有界传输

接收、发送缓存设有大小限制。慢客户端导致缓存超限时断开连接，避免无限积压。
ROS 桥接只保留最新遥测/图像和最新速度命令；重连后不重放旧命令。
新增高分辨率图像或三维点云时，应版本化协议并调整帧大小/吞吐限制，或使用独立二进制数据通道。



## v006: pointcloud and imu_sample

Clients opt into large pointcloud and high-rate inertial packets by sending capabilities ["pointcloud", "imu_sample"] in hello. Legacy clients continue receiving the v1 telemetry format. Pointcloud encoding xyz_irt_f32_u16_le: 24 bytes per point, x/y/z/intensity float32 at offsets 0/4/8/12, ring uint16 at16, padding at18, time float32 at20. 36 organized rows, default180columns, NaN XYZ for no return, snapshot=true and time=0. Payload is base64; stamp and position_ros describe the frozen scan origin. IMU samples include stamp, epoch, seq, orientation, angular_velocity, linear_acceleration and position_ros at200Hz. Default front mount is [1,0,2.65] in ROS body coordinates. Pausing stops both products; resetting starts a new epoch. The adapter publishes PointCloud2 /points_raw, NaN-filtered /points_lio and Imu /imu/data. See FUSION_README.md for clock, frames and fusion integration.


## v007 sensor mount correction

The lidar and IMU are mounted directly above the PTZ camera with no forward mast. The common sensor origin is now [0.335,0,2.55] metres in ROS body coordinates, replacing the v006 mount. Message fields, timestamps and lidar-to-IMU identity extrinsics are unchanged.


## v008 Ackermann chassis

cmd_vel linear_x is rear-axle longitudinal speed; angular_z is desired yaw rate. The simulator maps Twist to steering with atan(wheelbase*angular_z/linear_x), limits steering angle and rate, and reports actual motion. Zero linear speed cannot rotate the body. Reverse yaw retains ROS convention. Wheelbase=1.56m, track=1.82m, central steering limit=28deg, rate=55deg/s. steering contains model, central_angle_rad, max_angle_rad, actual_yaw_rate. visual_wheels adds steering_rad, front_wheels, centers_godot, wheelbase_m, track_m. Body-origin odometry includes the rear-axle lever-arm lateral velocity. Cruise target=36km/h; curved sections slow down.
