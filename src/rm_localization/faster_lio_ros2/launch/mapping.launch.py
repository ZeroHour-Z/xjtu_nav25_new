import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():


    params_file = os.path.join(
      get_package_share_directory('faster_lio_ros2'),'config','ros2.yaml')
    laser_mapping = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name='laser_mapping',
        parameters=[params_file],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(laser_mapping)

    return ld