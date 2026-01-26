from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    loop = LaunchConfiguration("loop")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    livox_custom_topic = LaunchConfiguration("livox_custom_topic")

    input_topic = LaunchConfiguration("input_topic")
    frame_id = LaunchConfiguration("frame_id")
    resolution = LaunchConfiguration("resolution")
    width_m = LaunchConfiguration("width_m")
    height_m = LaunchConfiguration("height_m")
    origin_x = LaunchConfiguration("origin_x")
    origin_y = LaunchConfiguration("origin_y")
    z_clip_min = LaunchConfiguration("z_clip_min")
    z_clip_max = LaunchConfiguration("z_clip_max")
    min_points_per_cell = LaunchConfiguration("min_points_per_cell")
    step_threshold_m = LaunchConfiguration("step_threshold_m")

    share_dir = get_package_share_directory("rm_ta")
    default_rviz_cfg = os.path.join(share_dir, "rviz", "traversability_default.rviz")

    # Resolve default bag path relative to this file (repo root/rosbags)
    script_dir = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(script_dir, "..", "..", "..", ".."))
    default_bag_dir = os.path.join(repo_root, "rosbags")

    return LaunchDescription(
        [
            # Common args
            DeclareLaunchArgument(
                "bag_path",
                default_value=default_bag_dir,
                description="Path to rosbag2 directory. Leave empty to skip playback.",
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Livox topics
            DeclareLaunchArgument(
                "livox_custom_topic",
                default_value="/livox/lidar",
                description="Livox custom message topic in bag or driver",
            ),
            # Traversability node args (same defaults as single-node launch)
            DeclareLaunchArgument(
                "input_topic",
                default_value="/cloud_registered_body",
                description="Input PointCloud2 topic",
            ),
            # DeclareLaunchArgument(
            #     "input_topic",
            #     default_value="/livox/points",
            #     description="Input PointCloud2 topic",
            # ),
            DeclareLaunchArgument(
                "frame_id", default_value="body", description="Output costmap frame ID"
            ),
            DeclareLaunchArgument(
                "gravity_frame",
                default_value="camera_init",
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
            # Static TF: map -> livox_frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_livox_frame_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "livox_frame"],
            ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="tf_body2base_link",
            #     arguments=[
            #         "0.0",
            #         "0.12848040398218347",
            #         "-0.2932452655927712",
            #         "1.5707963267948966",
            #         "0.2617993877991494",
            #         "0",
            #         "body",
            #         "base_link",
            #     ],
            # ),
            # Livox custom msg -> PointCloud2
            # Node(
            #     package="livox_to_pointcloud2",
            #     executable="livox_to_pointcloud2_node",
            #     name="livox_to_pointcloud2_node",
            #     output="screen",
            #     parameters=[
            #         {
            #             "use_sim_time": use_sim_time,
            #             "roll": 0.0,
            #             "pitch": 0.0,
            #             "yaw": 0.0,
            #         }
            #     ],
            #     remappings=[("/livox/lidar", livox_custom_topic)],
            # ),
            # Our traversability node
            Node(
                package="rm_ta",
                executable="traversability_costmap_node",
                name="traversability_costmap_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "frame_id": frame_id,
                        "resolution": resolution,
                        "width_m": width_m,
                        "height_m": height_m,
                        "origin_x": origin_x,
                        "origin_y": origin_y,
                        "z_clip_min": z_clip_min,
                        "z_clip_max": z_clip_max,
                        "min_points_per_cell": min_points_per_cell,
                        "step_threshold_m": step_threshold_m,
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
