# XJTU_nav2025

从ROS1迁移而来，保留了特色功能和通信模块。同时参考深北莫导航开源、中国科学技术大学导航开源等开源实现技术的迭代与进步。

## 已经实现的功能

### 1. 多传感器数据采集与处理
- **Livox激光雷达驱动支持**：集成livox_ros_driver2，实现高频点云数据采集
- **点云数据转换**：livox_to_pointcloud2模块，支持多种点云格式转换
- **IMU数据融合**：imu_complementary_filter实现IMU数据互补滤波
- **点云转激光雷达数据**：pointcloud_to_laserscan实现传感器数据格式转换

### 2. 高精度定位系统
- **Fast-LIO定位算法**：集成FAST_LIO和Point-LIO算法，实现高精度激光雷达惯性里程计
- **ROS2定位封装**：fast_lio_localization_ros2提供完整的定位ROS2接口
- **实时定位服务**：支持点云地图匹配定位，适用于室内外复杂环境

### 3. 地形感知与分析
- **实时地形分析**：rm_ta模块实现基于点云的可通行性分析
- **地面分割算法**：linefit_ground_segementation_ros2实现线拟合地面分割
- **动态代价地图生成**：支持实时地形代价地图构建和更新
- **RViz可视化集成**：提供地形分析结果的RViz可视化界面

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

### 6. 通信模块
- **ROS2通信框架**：rm_comm_ros2提供完整的ROS2通信解决方案
- **模块化设计**：支持灵活的通信拓扑结构配置

## 待实现的功能

### TODO
1. 导航点检测
2. 地形分析尚未完善(作为考核)
3. 通信和决策尚未进行适配
4. LOG和replay(作为考核)

### DONE

bfs脱困

## 配置环境

### rosdep 

```bash
rosdep init 
rosdep update
rosdep install --from-paths src --ignore-src -r -y  
```

### 启动

启动以下几个launch文件即可

1. 启动雷达驱动,发布点云
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 启动里程计和定位

> 迁移到ros2之后,point_lio的延迟变得大且不稳定,原因未知,暂时不建议在线跑。

```bash
ros2 launch rm_bringup SLAM_mapping.py backend:=point_lio # 仅建图
ros2 launch rm_bringup SLAM_odom_only.py backend:=faster_lio # 仅里程计
ros2 launch rm_bringup SLAM_and_localize.py backend:=faster_lio # 启动重定位和里程计
```

3. (实验性内容)启动地形分析,输出`/traversability/obstacles`和`/traversability/ground`

```bash
ros2 launch rm_ta bag_livox_ta.launch.py 
```

4. 启动`nav_stack`

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py # 还需要改进
```

或者使用仿真模式,可以用来测试决策

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

5. 启动决策(还没测):
```bash
ros2 launch rm_bt_decision bt.launch.py  
```

6. 启动通信节点

```bash
ros2 launch rm_comm_ros2 rm_comm_bringup.launch.py
```