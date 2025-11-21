#!/usr/bin/env python3
# coding: utf-8

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from datetime import datetime
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    # Arguments
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="point_lio",
        description="fast_lio | faster_lio | point_lio",
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    # No rosbag playback here; run rosbag externally if needed

    record_rosbag_arg = DeclareLaunchArgument(
        "record_rosbag",
        default_value="false",
        description="Record /livox/lidar and /livox/imu",
    )
    record_output_arg = DeclareLaunchArgument(
        "record_output",
        default_value="rosbags/mapping_record",
        description="Output prefix/path for rosbag record",
    )

    pcd_save_en_arg = DeclareLaunchArgument(
        "pcd_save_en", default_value="True", description="Override pcd_save.pcd_save_en"
    )
    pcd_save_interval_arg = DeclareLaunchArgument(
        "pcd_save_interval",
        default_value="-1",
        description="Override pcd_save.interval (-1 disables periodic save)",
    )

    LAUNCH_FILE = Path(__file__).resolve()  # 若为 symlink，会解到源码路径
    LAUNCH_DIR = LAUNCH_FILE.parent  # your_pkg/launch
    PKG_ROOT = LAUNCH_DIR.parent  # your_pkg （源码根，若 symlink-install）

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    tmp_path = PKG_ROOT / "tmp"
    tmp_path.mkdir(parents=True, exist_ok=True)

    default_pcd_path = tmp_path / f"scans_{ts}.pcd"
    print(f"Default PCD path: {default_pcd_path}")

    pcd_save_file_arg = DeclareLaunchArgument(
        "pcd_save_file",
        default_value=str(default_pcd_path),
        description="Path (absolute or package-relative) for final scans.pcd",
    )

    # Param files (defaults)
    fast_lio_params_arg = DeclareLaunchArgument(
        "fast_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "fast_lio_mid360.yaml",
            ]
        ),
        description="YAML for fast_lio node",
    )
    faster_lio_params_arg = DeclareLaunchArgument(
        "faster_lio_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "faster_lio_ros2.yaml",
            ]
        ),
        description="YAML for faster_lio_ros2 node",
    )
    point_lio_ros2_params_arg = DeclareLaunchArgument(
        "point_lio_ros2_params",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rm_bringup"),
                "config",
                "point_lio_mid360.yaml",
            ]
        ),
        description="YAML for point_lio_ros2 node",
    )

    # Configurations
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    record_rosbag = LaunchConfiguration("record_rosbag")
    record_output = LaunchConfiguration("record_output")
    pcd_save_en = LaunchConfiguration("pcd_save_en")
    pcd_save_interval = LaunchConfiguration("pcd_save_interval")
    pcd_save_file = LaunchConfiguration("pcd_save_file")

    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")

    # Small helper to build equality condition "var == value"
    def equals(var, value):
        return PythonExpression(["'", var, "' == '", value, "'"])

    # Common parameter bundle used by all backends
    common_params = {
        "use_sim_time": use_sim_time,
        "pcd_save": {
            "pcd_save_en": pcd_save_en,
            "interval": pcd_save_interval,
            "file_path": pcd_save_file,
        },
    }

    # Backend nodes (mapping only)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params, common_params],
        condition=IfCondition(equals(backend, "fast_lio")),
    )

    faster_lio_node = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name="laser_mapping",
        output="screen",
        parameters=[faster_lio_params, common_params],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "faster_lio")),
    )

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[point_lio_ros2_params, common_params],
        condition=IfCondition(equals(backend, "point_lio")),
    )

    # Optional recording of Live/Bag topics
    bag_record_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            record_output,
            "/livox/lidar",
            "/livox/imu",
        ],
        output="screen",
        condition=IfCondition(record_rosbag),
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory("rm_bringup"), "rviz", "mapping.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            # Args
            backend_arg,
            rviz_arg,
            use_sim_time_arg,
            record_rosbag_arg,
            record_output_arg,
            pcd_save_en_arg,
            pcd_save_interval_arg,
            pcd_save_file_arg,
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            # Mapping backends
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            # Data source/recording
            bag_record_process,
            # RViz
            rviz_node,
        ]
    )
