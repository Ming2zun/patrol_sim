# 棉田项目移植说明

从棉田四轮项目复制了 KISS-ICP 及其 vendor、80 m 体素点云累积、RViz 配置和 test_c/LineDrawTool。相关复制代码保留 Apache-2.0 / MIT 许可，见 LICENSE.apache-2.0 和 alpine_sim_slam/vendor。

- 雪山没有双 RTK 数据，使用仿真真值里程计作为 KISS-ICP 外部先验。它不是独立激光定位，也没有回环优化；roll/pitch/z 依赖真值先验，ICP 只进行有界的局部平面修正。
- `/points_lio`：36×180，目标 10 Hz；`/imu/data`：目标 200 Hz；都使用仿真时间。所有新节点和 RViz 设置 use_sim_time=true。
- `/sim/localization/odom` 是仿真真值，frame=map、child=base_link。`/sim/slam/odom` 是 KISS-ICP 输出，frame=map、child=slam_base_link。只有桥接发布 map→base_link，避免双重 TF。
- 雷达位于 base_link 的 [0.335,0,2.55] m；SLAM 的 lidar_position_ros 参数与桥接/仿真配置必须一致。
- RViz 默认显示车辆 MarkerArray、SLAM 路径、规划路径和 `/sim/segmentation/accumulated_points`。累积云保留车周围80米、0.25米体素，1 Hz刷新，保留地形高度，不套用平地的0.15米绝对高度裁剪。
- LineDraw 插件的打点/发布/清除接到 `/sim/path`，通过 `/sim/set_remote` 接管仿真，跟踪器输出 `/cmd_vel`。默认1m/s，按阿克曼约束限曲率，不沿用四轮独立转向控制器，不发原地旋转命令。急停/暂停/断连/复位清除跟踪路径。
- 插件仍是 map 平面打点，尚未做山地路径贴地、地形可通行判断或自动绕障；请在平坦可行驶路段测试。
- 棉田项目的源码、运行工作空间均未修改。
