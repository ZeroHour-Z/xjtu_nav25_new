---
title: "哨兵寄录"
publishDate: 2025-10-05
updatedDate: 2025-02-01
description: "调车寄录, changelog, 算法探索"
tags:
  - SLAM
  - navigation
  - fast-lio

---
# 哨兵寄录

## 260208 调试

- 追击决策测试成功
目前的追击（test.yaml）：电控发来standby，什么都不做；电控发attack或pursiut，追击电控发来的敌方坐标（odom系）；电控发patrol，进行原地巡逻。与对方保持距离的方式：电控端控制，如果距离在1.5到3.5米之间，电控发车当前的位置（odom系），即原地不动，若超出这个范围就会发送目标位置或者目标的相反位置。

## 260201 调试

- 更新了功能包命名
- 代码格式化，添加bringup，修改命名

## 260130 调试

./src/rm_perception/rm_terrain_analysis/下新写了个功能包叫region_detector，目前只是负责过颠簸路段，其实并不是基于雷达检测点云判读的地形，实际是在config中手动设置区域参数，将某一部分区域作为颠簸路段，在规划的路径通过该区域时，提前修正好位姿，正对着通过

## 260129 调试

**done：**

- 陀螺移动下的定点导航
- 动态避障功能

**todo：**

- 避障参数待优化
- 导航参数待优化
- 过起伏路段

## 260126 调试

重构坐标系ID并优化初始化参数：
- SLAM和"camera_init"更改为"body"和"odom"，确保系统一致性（本质上nav2所需的odom和fastlio的camera_init是一个东西）
- 增加全局定位参数配置，新增初始位的坐标系ID"livox_frame"和"camera_init"更改为"body"和"odom"，确保系统一致性（本质上nav2所需的odom和fastlio的camera_init是一个东西）
- 增加全局定位参数配置，新增初始位姿设置及多角度假设初始化选项，优化了定位
- 将点云处理中的消息指针类型从ConstPtr调整为ConstSharedPtr，提升内存管理效率，支持ROS2
- 更改部分宏定义之后加分号的会引起警告的写法（宏本身展开自带分号）
- 更改部分注释号使用不规范的问题
- 修复了SLAM_and_localize backen:=faster_lio|fast_lio|point_lio

## 260124 调试

- 建图，重定位基础功能已经调完
- 解决了libusb与MVS SDK的冲突，解决方法是在./zshrc里直接加`export LD_LIBRARY_PATH=""`,强制给LD_LIBRARY_PATH赋空值
- 解决了SLAM_and_localize中的各问题
- 通信和电控对完

## 251121 调试

收到电控消息sentry_state为150, 0x96, 但有效范围为0-9, 何意味
无语了，没对齐

## 251119 调试

操作！！！

1. 启动雷达驱动,发布点云

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 启动里程计和定位

> 迁移到ros2之后,point_lio的延迟变得大且不稳定,原因未知,暂时不建议在线跑。

```bash
ros2 launch rm_bringup slam_mapping_only.launch.py backend:=point_lio rviz:=true # 仅建图
ros2 launch rm_bringup slam_odom_only.launch.py backend:=faster_lio rviz:=true # 仅里程计
ros2 launch rm_bringup slam_and_localize.launch.py backend:=faster_lio rviz:=true # 启动重定位和里程计
```

3. (实验性内容)启动地形分析,输出`/traversability/obstacles`和`/traversability/ground`

```bash
ros2 launch rm_terrain_analysis bag_livox_terrain_analysis.launch.py 
```

4. 启动`nav_stack`

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py # 还需要改进
```

或者使用仿真模式,可以用来测试决策

```bash
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

启动决策(还没测):

```bash
ros2 launch rm_decision bt.launch.py  
```

5. 启动通信节点

```bash
ros2 launch rm_comm_ros2 rm_comm_bringup.launch.py
```

## ROS 服务器配置，YAML 中不写坐标

ros2 param set /rm_bt_decision_node supply.heal_x -2.0
ros2 param set /rm_bt_decision_node supply.heal_y -2.0
ros2 param set /rm_bt_decision_node supply.reload_x -2.0
ros2 param set /rm_bt_decision_node supply.reload_y -2.0

## 状态传递到决策树：rm_comm_ros2/src/handler_node.cpp

> 99 行handleNavCommand，去掉注释

**状态转换和参数发布：**

紧着到 104 行

## 关于 rm_comm_bringup.launch.cpp

> 第11行传入参数应为实数，传入 int 会出错

## 关于 global_velocity_controller_node.cpp

> 注释掉硬编码的测试代码，使用PID控制器计算的速度

```cpp
// 290行左右
vx_map_cmd = 1.0;
vy_map_cmd = 0.0;
predicted_yaw = current_yaw + 0.1 + 0.0 * std::hypot(vx_map_cmd, vy_map_cmd);
```

## 项目 Launch 文件：

1. 传感器驱动模块（rm_driver）

```bash
# Livox激光雷达驱动 (ROS1版本)
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

2. 定位模块（rm_localization）

```bash
# 统一定位启动 (推荐)
ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=fast_lio
ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=faster_lio
ros2 launch rm_localization_bringup localization_bringup.launch.py backend:=point_lio
```

3. 感知模块（rm_perception）

```bash
# 地面分割
ros2 launch linefit_ground_segmentation_ros segmentation.launch.py
ros2 launch linefit_ground_segmentation_ros test.launch.py

# 地形分析
ros2 launch rm_ta traversability_costmap.launch.py
ros2 launch rm_ta bag_livox_ta.launch.py

# IMU滤波
ros2 launch imu_complementary_filter complementary_filter.launch.py
```

4. 导航模块（rm_nav）

```bash
# 完整Nav2栈
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py

# 全局速度控制器
ros2 launch global_velocity_controller global_velocity_controller.launch.py
ros2 launch global_velocity_controller global_velocity_controller_no_sim.launch.py
```

5. 行为树模块（rm_bt_decision）

```bash
# 行为树决策
ros2 launch rm_bt_decision bt.launch.py
```

6. 通信模块（rm_comm_ros2）

```bash
# 通信框架
ros2 launch rm_comm_ros2 rm_comm_bringup.launch.py
```

## 启动

### 完整系统启动

```bash
# 1. 启动雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 启动定位
ros2 launch rm_localization_bringup SLAM_and_localization.launch.py backend:=point_lio rviz:=true
ros2 launch rm_localization_bringup SLAM_and_odom.launch.py backend:=point_lio rviz:=false

# 3. 启动导航
ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py

# 4. 启动决策
ros2 launch rm_bt_decision bt.launch.py
```

### 测试和开发

```bash
# 单独测试定位
ros2 launch fast_lio mapping.launch.py

# 仿真测试导航，测决策
ros2 launch nav2_client_cpp nav2_stack_with_gvc_sim.launch.py
```

## Launch文件参数说明

定位模块参数

```bash
# 选择算法后端
backend:=fast_lio | faster_lio | point_lio

# 启用RViz
rviz:=true | false

# 使用仿真时间
use_sim_time:=true | false

# 参数文件
fast_lio_params:=/path/to/config.yaml
```

导航模块参数

```bash
# 地图文件
map_yaml:=/path/to/map.yaml

# Nav2参数
params_file:=/path/to/nav2_params.yaml

# 行为树文件
default_bt_xml:=/path/to/bt.xml
```

决策模块参数

```bash
# 行为树文件
tree:=/path/to/bt.xml

# 行为树频率
tick_hz:=10.0

# 是否启用web可视化
use_web_viewer:=true | false
```