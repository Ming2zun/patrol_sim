# 许可与来源

项目新增代码与棉田移植代码按顶层 Apache-2.0 提供；原 ROS 桥接保留其 MIT 声明，第三方代码不受顶层许可替代。

- KISS-ICP、Sophus、robin-map：来自棉田项目的vendor快照，含已有PosePrior修改；版权和MIT许可文件保留在 ros2/src/alpine_sim_slam/vendor/。
- test_c/LineDrawTool 及移植节点：棉田项目，Apache-2.0；相关说明在各ROS包中。
- LIO-SAM 可选配置：原包保留BSD-3-Clause许可证；算法本体不包含在仓库内。
- Godot 引擎：MIT，完整MIT文本见 licenses/Godot.txt，官方 [许可及第三方说明](https://godotengine.org/license/)。发布运行包时一并提供引擎许可，不把它当成本项目原创代码。
- Noto Sans SC：SIL OFL 1.1，完整文本见 licenses/OFL.txt，素材包保留原字体许可。
- Poly Haven模型、贴图和HDR：CC0，素材包 assets/pbr/sources*.json 与 assets/vegetation/sources.json 保留具体来源。
- 车辆及原场景来自项目交付资产；本文件不额外授予用户参考图或外来素材的再分发权。历史来源说明存档在 docs/ASSET_PROVENANCE.md。

GitHub源码包不包含大素材和引擎二进制。发布者应把素材来源和许可随独立下载包一起提供。
