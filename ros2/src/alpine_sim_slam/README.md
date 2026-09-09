# Alpine KISS-ICP

复制自棉田项目，保留 KISS-ICP / Sophus / robin-map 的许可及 PosePrior 修改。

输入 `/points_lio`、`/sim/localization/odom`、`/sim/status`。以仿真里程计作为先验，不是独立SLAM。使用map/base_link与lidar36_link，默认安装位移[0.335,0,2.55]米、60米范围、0.25米体素、2个工作线程；快照雷达不去畸变。位置时间匹配容差0.12秒，超过2秒的扫描不处理。

输出 `/sim/slam/odom`、`/sim/slam/registered_points`、`/sim/slam/local_map`、`/sim/slam/path`；不发布与车辆竞争的TF。复位或时间回退清除历史。ICP只做有界平面修正，姿态和高度由先验提供；没有IMU融合、回环或全局优化。
