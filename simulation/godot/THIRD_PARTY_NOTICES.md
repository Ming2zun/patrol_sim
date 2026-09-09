# 第三方组件

## Godot Engine 4.7.2 stable
MIT License。官方来源：https://godotengine.org/ 与 https://github.com/godotengine/godot
编辑器 SHA-256 已与官方发布元数据核对。
Windows / Linux 发布模板来自 Godot 官方下载服务器，提取时校验了 ZIP CRC 与未压缩大小。
完整 Godot 许可证见 tools/GODOT_LICENSE.txt；引擎的进一步版权说明可查 https://godotengine.org/license/ 。

## Noto Sans SC
Copyright Noto / Adobe / Google font contributors，SIL Open Font License 1.1。
字体来自 google/fonts 的 ofl/notosanssc；许可文本位于 godot_project/assets/fonts/OFL.txt。

## 本项目资产
三维场景和车辆由本项目现有 Blender 文件导出。此说明不对用户提供的参考图或资产授予额外再分发权利。
新增桥接与仿真源代码交付给本项目用户；ROS 包声明为 MIT。



## Poly Haven — rendering v002

PBR textures and HDR panorama from https://polyhaven.com/ are CC0 (https://polyhaven.com/license). Exact assets and download URLs are recorded in godot_project/assets/pbr/sources.json and sources_additional.json and copied with the release. The tree impostor is rendered from the existing project model.


## Poly Haven — vegetation v005

CC0-1.0 models: fir_tree_01, tree_small_02, shrub_01 and shrub_03. Source links and selected variants: godot_project/assets/vegetation/sources.json (also distributed in licenses/vegetation_sources.json). Geometry was simplified for real-time use; shrubs were regrouped, and snowy materials and billboards were derived for this project. Dense pine derives from the original project model. https://polyhaven.com/license


## v006 new vegetation and LIO-SAM configuration

Additional CC0 Poly Haven assets: pine_tree_01, shrub_02, shrub_04, fern_02. The twin-stem broadleaf is derived from tree_small_02. Sources: godot_project/assets/vegetation/sources.json. LIO-SAM ROS 2 configuration is adapted from https://github.com/TixiaoShan/LIO-SAM/tree/ros2 under BSD-3-Clause; see ros2_bridge/godot_ros2_bridge/LIO_SAM_LICENSE.txt. The algorithm is an optional external Linux dependency, not included in the simulator executable.
