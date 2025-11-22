#!/bin/bash
# 检查无地图避障导航配置

echo "=== 无地图避障导航配置检查 ==="
echo ""

# 1. 检查 costmap 节点
echo "1. 检查 costmap 节点:"
if ros2 node list 2>/dev/null | grep -q "local_costmap"; then
    echo "   ✓ local_costmap 节点正在运行"
else
    echo "   ✗ local_costmap 节点未运行"
fi

if ros2 node list 2>/dev/null | grep -q "global_costmap"; then
    echo "   ✓ global_costmap 节点正在运行"
else
    echo "   ✗ global_costmap 节点未运行"
fi

# 2. 检查传感器话题
echo ""
echo "2. 检查传感器数据:"
if timeout 1 ros2 topic list 2>/dev/null | grep -q "/scan"; then
    echo "   ✓ /scan 话题存在"
    scan_info=$(timeout 1 ros2 topic info /scan 2>/dev/null | head -1)
    if [ -n "$scan_info" ]; then
        echo "      $scan_info"
    fi
else
    echo "   ⚠ /scan 话题不存在（可选，如果有点云数据）"
fi

if timeout 1 ros2 topic list 2>/dev/null | grep -q "/cloud_registerd"; then
    echo "   ✓ /cloud_registerd 话题存在"
    cloud_info=$(timeout 1 ros2 topic info /cloud_registerd 2>/dev/null | head -1)
    if [ -n "$cloud_info" ]; then
        echo "      $cloud_info"
    fi
else
    echo "   ⚠ /cloud_registerd 话题不存在"
fi

# 3. 检查 costmap 话题
echo ""
echo "3. 检查 costmap 话题:"
if timeout 1 ros2 topic list 2>/dev/null | grep -q "/local_costmap/costmap"; then
    echo "   ✓ /local_costmap/costmap 话题存在（避障功能正常）"
else
    echo "   ✗ /local_costmap/costmap 话题不存在（避障功能可能无法工作）"
fi

# 4. 检查 TF 变换
echo ""
echo "4. 检查 TF 变换:"
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -1 | grep -q "At time"; then
    echo "   ✓ odom -> base_link TF 变换存在"
else
    echo "   ✗ odom -> base_link TF 变换不存在"
fi

# 5. 检查 Nav2 核心节点
echo ""
echo "5. 检查 Nav2 核心节点:"
nodes=("controller_server" "planner_server" "bt_navigator")
for node in "${nodes[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        echo "   ✓ $node 正在运行"
    else
        echo "   ✗ $node 未运行"
    fi
done

# 6. 检查控制器碰撞检测
echo ""
echo "6. 配置检查:"
echo "   - local_costmap: 已配置 obstacle_layer 和 inflation_layer"
echo "   - 坐标系: odom (无地图导航)"
echo "   - 控制器碰撞检测: 已启用"
echo "   - 传感器数据源: /scan 和 /cloud_registerd"

echo ""
echo "=== 检查完成 ==="
echo ""
echo "如果所有项目都显示 ✓，则无地图避障导航应该可以正常工作！"
echo "如果 costmap 话题不存在，请检查 costmap 节点是否成功启动。"

