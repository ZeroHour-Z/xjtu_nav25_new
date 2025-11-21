# nav2_client_cpp 使用说明

本包提供：
- **navigate_to_pose_node**：简单的 C++ 动作客户端，调用 Nav2 的 `NavigateToPose` 行为
- **path_follower_node**：高级路径跟随节点，支持路径规划、实时控制、速度平滑和重新规划
- Nav2 参数配置（全局规划器 Navfn、控制器 RPP、默认恢复行为与行为树）
- 启动文件：
  - `nav2_stack_with_gvc_sim.launch.py`：启动完整 Nav2 栈，集成全局速度控制器和 RViz 可视化

前置条件：已具备 `map -> base_link` 的 TF（你已说明存在），并有地图与定位来源（如 SLAM 或 map_server+AMCL）。

## 功能说明

### navigate_to_pose_node
- 调用 Nav2 的 NavigateToPose 动作服务
- 支持通过参数设置目标点位置和朝向
- 提供动作反馈和结果回调
- 完成后自动关闭节点

### path_follower_node
- 调用 Nav2 的 ComputePathToPose 动作进行路径规划
- 实现实时路径跟随控制
- 支持速度平滑和加速度限制
- 可从 RViz 的 2D Nav Goal 工具接收目标点
- 支持定时和按需重新规划
- 发布目标速度命令到指定话题

## 依赖
- ROS 2 Nav2 组件：`nav2_planner`、`nav2_controller`、`nav2_bt_navigator`、`nav2_behaviors`、`nav2_lifecycle_manager`
- 插件：`nav2_navfn_planner`、`nav2_regulated_pure_pursuit_controller`、`nav2_costmap_2d`、`nav2_map_server`
- 外部依赖：`global_velocity_controller`
- 传感器：默认局部代价地图订阅 `/scan`（LaserScan）

## 构建
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select nav2_client_cpp
source install/setup.bash
```

## 参数文件
默认参数位于：`config/nav2_params.yaml`
- 全局规划器：`nav2_navfn_planner/NavfnPlanner`
- 控制器：`nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController`
- 行为树：默认 `nav2_bt_navigator` 内置 `navigate_w_replanning_and_recovery.xml`
- 代价地图：全局（静态层+膨胀层）、局部（障碍层+膨胀层），局部订阅 `/scan`

如需修改，可直接编辑 YAML 或在运行时通过 launch 覆盖。

## 启动完整 Nav2 栈（推荐）
```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```
此启动文件包含：
- Nav2 核心节点（规划器、控制器、行为服务器、BT导航器）
- 地图服务器（使用预定义地图）
- 全局速度控制器（模拟模式）
- 静态 TF：map -> odom
- RViz 可视化（支持通过界面设置导航目标）

## 仅发送目标点（使用 navigate_to_pose_node）
```bash
ros2 run nav2_client_cpp navigate_to_pose_node --ros-args \
  -p goal_x:=1.0 \
  -p goal_y:=0.5 \
  -p goal_yaw:=1.57 \
  -p goal_frame_id:=map
```

## 路径跟随模式（使用 path_follower_node）
```bash
ros2 run nav2_client_cpp path_follower_node --ros-args \
  -p goal_x:=2.0 \
  -p goal_y:=1.0 \
  -p goal_yaw:=0.0 \
  -p desired_speed:=0.6 \
  -p target_twist_map_topic:=/target_twist_map
```

### path_follower_node 参数说明
- `goal_x`, `goal_y`, `goal_yaw`：目标点位置和朝向
- `goal_frame_id`：目标点坐标系（默认 "map"）
- `desired_speed`：期望速度（m/s）
- `lookahead_distance`：前瞻距离（m）
- `replan_period`：重新规划周期（s）
- `goal_tolerance`：目标点容差（m）
- `target_twist_map_topic`：目标速度发布话题
- `enable_lowpass_smoothing`：启用低通滤波平滑
- `smoothing_alpha`：平滑系数（0-1，越小越平滑）
- `enable_slew_limit`：启用加速度限制
- `max_linear_accel`：最大线加速度（m/s²）
- `max_angular_accel`：最大角加速度（rad/s²）

## 在 RViz 中设置目标点
1. 启动 RViz（通过启动文件自动启动）
2. 选择 "2D Nav Goal" 工具
3. 在地图上点击目标位置并拖拽设置朝向
4. path_follower_node 将自动接收目标并开始导航

## 常见问题
- 若曾用 `--merge-install`，请勿与默认 isolated 混用；切换需清理 `build/ install/ log/`。
- 反馈显示 `Nav2 action server not available`：确认 `bt_navigator` 已激活（生命周期），以及 `lifecycle_manager` 的 autostart 为 true。
- 无法规划：确认 `/map`、TF 与代价地图传感器话题是否正常；必要时调整 `nav2_params.yaml` 中 costmap 分辨率与膨胀半径。
- 地图服务器启动失败：检查 `map_yaml` 路径是否存在且文件有效。
- 全局速度控制器错误：确保 `global_velocity_controller` 包已正确安装和配置。
