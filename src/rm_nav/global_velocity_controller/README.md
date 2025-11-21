# global_velocity_controller 使用说明

- 以 `map` 坐标系为参考，针对全向底盘的速度闭环控制（PID）。
- 订阅全局期望速度：`/target_twist_map`（`geometry_msgs/TwistStamped`，`header.frame_id`=map）。
- 订阅里程计：`/odom`（提供当前速度）。
- 发布底盘控制：`/cmd_vel`（`geometry_msgs/Twist`，在 `base_link` 体坐标下）。

## 构建
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select global_velocity_controller
source install/setup.bash
```

## 运行
- 直接运行（带默认参数）：
```bash
ros2 launch global_velocity_controller global_velocity_controller.launch.py
```
- 指定参数文件：
```bash
ros2 launch global_velocity_controller global_velocity_controller.launch.py \
  params_file:=/home/you/controller_params.yaml
```
- 手动加载参数文件：
```bash
ros2 run global_velocity_controller global_velocity_controller_node --ros-args \
  --params-file $(ros2 pkg prefix global_velocity_controller)/share/global_velocity_controller/config/controller_params.yaml
```

## 发布期望速度（在map系）
```bash
ros2 topic pub -r 10 /target_twist_map geometry_msgs/TwistStamped "{header: {frame_id: map}, twist: {linear: {x: 0.3, y: 0.0}, angular: {z: 0.0}}}"
```

## 参数
见 `config/controller_params.yaml`。 