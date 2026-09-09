# CottonSim LineDraw tool

Adapted from `KaiwuOS-Control-H/src/test_c` and its `odomtest/config/rviz2.rviz` configuration. Retains the `test_c/LineDrawTool` plugin identity, continuous XY-plane picking, rounded corners, interpolated Path requests, preview markers, and 打点 / 发布 / 清除 dock panel.

CottonSim adaptation: default corner radius 1.5 m (editable down to zero), sample step 0.2 m, fixed frame must be map, no unused mpc_msgs/Lane or implement controls, clear publishes an empty request to cancel tracking. The request adapter consumes `/test_c/click_path_request` and starts the four-wheel controller through `/sim/path`. Commands are volatile so restarting RViz cannot replay a retained publish command. This adaptation is distributed under the repository Apache-2.0 license.
