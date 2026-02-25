# XJTU nav_26

## 配置环境

### rosdep 

```bash
rosdep init 
rosdep update
rosdep install --from-paths src --ignore-src -r -y  
./build.sh
```

## 快速开始

### bringup启动
我们提供了一个统一的启动入口，支持导航和建图两种模式。

#### 1. 导航模式 (默认)
启动所有核心模块：驱动、定位(LIO+重定位)、Nav2导航栈、地形分析（避障、标记颠簸路段）、决策、通信。这些模块都可以在sentry_bringup.launch.up中修改参数为true或false来控制是否启动
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=nav
```
*   `rviz:=true` (默认): 打开 RViz 可视化界面。
*   `map:=/path/to/map.yaml`: 指定加载的地图（默认为 `src/rm_bringup/PCD/test4/newMap.yaml`）。

#### 2. 建图模式
仅启动 LiDAR 驱动和 LIO 建图后端。
```bash
ros2 launch rm_bringup sentry_bringup.launch.py mode:=mapping
```

### 依次启动

1. 启动雷达驱动,发布点云
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 启动里程计和定位

```bash
ros2 launch rm_bringup slam_mapping_only.launch.py backend:=point_lio # 仅建图
ros2 launch rm_bringup slam_odom_only.launch.py backend:=faster_lio # 仅里程计
ros2 launch rm_bringup slam_and_localize.launch.py backend:=faster_lio # 启动重定位和里程计
```

3. 启动地形分析,输出`/traversability/obstacles`和`/traversability/ground`

```bash
ros2 launch rm_terrain_analysis traversability_pointcloud.launch.py # 动态避障，若不启动就只有静态地图
ros2 launch rm_terrain_analysis region_detector.launch.py # 标记颠簸路段，以调整姿态通过
```

4. 启动导航栈

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py # 仿真模式
```

5. 启动决策:
```bash
ros2 launch rm_decision bt.launch.py # 目前测试了追击
```

6. 启动通信节点

```bash
ros2 launch rm_communication communication_bringup.launch.py
```

## 已经实现的功能

### 1. 多传感器数据采集与处理
- **Livox激光雷达驱动支持**：集成livox_ros_driver2，实现高频点云数据采集
- **点云数据转换**：livox_to_pointcloud2模块，支持多种点云格式转换
- **点云转激光雷达数据**：pointcloud_to_laserscan实现传感器数据格式转换

### 2. 高精度定位系统
- **Fast-LIO定位算法**：集成FAST_LIO和Point-LIO算法，实现高精度激光雷达惯性里程计
- **实时定位服务**：支持点云地图匹配定位

### 3. 地形感知与分析
- **实时地形分析**：rm_terrain_analysis模块实现基于点云的可通行性分析
- **地面分割算法**：linefit_ground_segmentation实现线拟合地面分割

### 4. 行为树决策框架
- **PyTrees行为树**：基于py_trees的ROS2行为树框架
- **灵活配置方式**：支持YAML配置和Python直写两种方式构建行为树
- **Nav2动作适配**：无缝集成NavigateToPose等Nav2导航动作
- **Web可视化**：支持py_trees_js Web界面实时监控行为树状态
- **补给管理功能**：内置SupplyManagerAction实现智能补给策略

### 5. 导航控制系统
- **Nav2客户端**：nav2_client_cpp提供C++导航客户端接口
- **全局速度控制器**：global_velocity_controller实现全局路径速度规划和loopback的Odom仿真
- **交互式障碍物管理**：rviz_click_obstacles支持在RViz中点击设置障碍物
- **动态障碍物层**：click_obstacles_layer实现动态障碍物地图层管理
