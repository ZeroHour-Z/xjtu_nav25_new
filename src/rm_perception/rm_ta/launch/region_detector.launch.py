"""
区域检测节点 launch 文件
用于检测机器人是否在特殊区域（如颠簸路段）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    # 参数文件路径
    config_dir = '/home/xjturm/xjtu_nav25_new/src/rm_perception/rm_ta/config/region_detector.yaml'
    
    region_detector_node = Node(
        package='rm_ta',
        executable='region_detector_node',
        name='region_detector_node',
        output='screen',
        parameters=[config_dir]
    )

    return LaunchDescription([
        region_detector_node,
    ])
