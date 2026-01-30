from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    input_topic = LaunchConfiguration("input_topic")
    frame_id = LaunchConfiguration("frame_id")
    gravity_frame = LaunchConfiguration('gravity_frame')
    resolution = LaunchConfiguration("resolution")
    width_m = LaunchConfiguration("width_m")
    height_m = LaunchConfiguration("height_m")
    origin_x = LaunchConfiguration("origin_x")
    origin_y = LaunchConfiguration("origin_y")
    z_clip_min = LaunchConfiguration("z_clip_min")
    z_clip_max = LaunchConfiguration("z_clip_max")
    min_points_per_cell = LaunchConfiguration("min_points_per_cell")
    step_threshold_m = LaunchConfiguration("step_threshold_m")
    step_max_threshold_m = LaunchConfiguration("step_max_threshold_m")
    density_min_pts_per_m3 = LaunchConfiguration("density_min_pts_per_m3")
    min_points_for_density = LaunchConfiguration("min_points_for_density")

    share_dir = get_package_share_directory("rm_ta")
    default_rviz_cfg = os.path.join(share_dir, "rviz", "traversability_default.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Traversability node args (same defaults as single-node launch)
            DeclareLaunchArgument(
                "input_topic",
                default_value="/cloud_registered_body",
                description="Input PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "frame_id", default_value="body", description="Output costmap frame ID"
            ),
            DeclareLaunchArgument(
                "gravity_frame",
                default_value="odom",
                description="Output costmap frame ID",
            ),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("width_m", default_value="10.0"),
            DeclareLaunchArgument("height_m", default_value="10.0"),
            DeclareLaunchArgument("origin_x", default_value="-5.0"),
            DeclareLaunchArgument("origin_y", default_value="-5.0"),
            DeclareLaunchArgument("z_clip_min", default_value="-0.3"),
            DeclareLaunchArgument("z_clip_max", default_value="1.0"),
            DeclareLaunchArgument("min_points_per_cell", default_value="3"),
            DeclareLaunchArgument("step_threshold_m", default_value="0.1"),
            DeclareLaunchArgument("step_max_threshold_m", default_value="2.0"),
            DeclareLaunchArgument("density_min_pts_per_m3", default_value="20.0"),
            DeclareLaunchArgument("min_points_for_density", default_value="3"),
            # Static TF: map -> odom -> body (仅用于无SLAM时的测试)
            # 注意：如果同时运行SLAM，SLAM会提供odom->body的动态TF，需要注释掉下面两个静态TF
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="map_to_odom_tf",
            #     arguments=["--x", "0", "--y", "0", "--z", "0", 
            #               "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            #               "--frame-id", "map", "--child-frame-id", "odom"],
            # ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="odom_to_body_tf",
            #     arguments=["--x", "0", "--y", "0", "--z", "0", 
            #               "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            #               "--frame-id", "odom", "--child-frame-id", "body"],
            # ),
            Node(
                package="rm_ta",
                executable="traversability_costmap_node",
                name="traversability_costmap_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "frame_id": frame_id,
                        'gravity_frame': gravity_frame,
                        "resolution": resolution,
                        "width_m": width_m,
                        "height_m": height_m,
                        "origin_x": origin_x,
                        "origin_y": origin_y,
                        "z_clip_min": z_clip_min,
                        "z_clip_max": z_clip_max,
                        "min_points_per_cell": min_points_per_cell,
                        "step_threshold_m": step_threshold_m,
                        "step_max_threshold_m": step_max_threshold_m,
                        "density_min_pts_per_m3": density_min_pts_per_m3,
                        "min_points_for_density": min_points_for_density,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            # RViz2 with default config
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", default_rviz_cfg],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
