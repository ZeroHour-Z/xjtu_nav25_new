#!/usr/bin/env python3
# coding: utf-8
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from datetime import datetime
from pathlib import Path
import os


def generate_launch_description():
    # Core arguments
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="point_lio",
        description="Backend: fast_lio | faster_lio | point_lio",
    )
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    # LAUNCH_FILE = Path(__file__).resolve()
    # PKG_ROOT = LAUNCH_FILE.parent.parent
    # (PKG_ROOT / "tmp").mkdir(parents=True, exist_ok=True)

    # Backend parameter files (defaults)
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
    fast_lio_params = LaunchConfiguration("fast_lio_params")
    faster_lio_params = LaunchConfiguration("faster_lio_params")
    point_lio_ros2_params = LaunchConfiguration("point_lio_ros2_params")

    # Helper for equality conditions
    def equals(var, value):
        return PythonExpression(["'", var, "' == '", value, "'"])

    # Backend odometry nodes (no relocalization)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[fast_lio_params],
        condition=IfCondition(equals(backend, "fast_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )
    faster_lio_node = Node(
        package="faster_lio_ros2",
        executable="run_mapping_online",
        name="laser_mapping",
        output="screen",
        parameters=[faster_lio_params],
        remappings=[("/Odometry", "/odom")],
        condition=IfCondition(equals(backend, "faster_lio")),
        # Fix libusb conflict with MVS SDK - prioritize system libusb
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')},
    )
    # point_lio_ros2_node = Node(
    #     package="point_lio_ros2",
    #     executable="pointlio_mapping",
    #     name="pointlio_mapping",
    #     output="screen",
    #     remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    #     parameters=[point_lio_ros2_params],
    #     condition=IfCondition(equals(backend, "point_lio")),
    # )
    point_lio_ros2_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("aft_mapped_to_init", "/odom"),
        ],
        parameters=[point_lio_ros2_params],
        condition=IfCondition(equals(backend, "point_lio")),
    )
    # Default static transform map3d -> camera_init (identity)
    tf_map3dto2d = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3dto2d",
        arguments=["0", "0", "0.24", "-1.5707963267948966", "0", "0", "map", "map3d"],
    )

    tf_map3d_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map3d_to_camera_init",
        arguments=["0", "0", "0", "0", "0", "0", "map3d", "camera_init"],
    )

    tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_body2base_link",
        arguments=[
            "0.0",
            "0.12",
            "-0.28",
            "1.5707963267948966",
            "0.27",
            "0",
            "body",
            "base_link",
        ],
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory("rm_bringup"), "rviz", "localize.rviz"
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
            fast_lio_params_arg,
            faster_lio_params_arg,
            point_lio_ros2_params_arg,
            # Nodes
            fast_lio_node,
            faster_lio_node,
            point_lio_ros2_node,
            tf_map3dto2d,
            tf_map3d_to_camera_init,
            tf_body2base,
            rviz_node,
        ]
    )
