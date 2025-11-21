from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    frame_id = LaunchConfiguration('frame_id')
    resolution = LaunchConfiguration('resolution')
    width_m = LaunchConfiguration('width_m')
    height_m = LaunchConfiguration('height_m')
    origin_x = LaunchConfiguration('origin_x')
    origin_y = LaunchConfiguration('origin_y')
    z_clip_min = LaunchConfiguration('z_clip_min')
    z_clip_max = LaunchConfiguration('z_clip_max')
    min_points_per_cell = LaunchConfiguration('min_points_per_cell')
    step_threshold_m = LaunchConfiguration('step_threshold_m')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/livox/points'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('width_m', default_value='20.0'),
        DeclareLaunchArgument('height_m', default_value='20.0'),
        DeclareLaunchArgument('origin_x', default_value='-10.0'),
        DeclareLaunchArgument('origin_y', default_value='-10.0'),
        DeclareLaunchArgument('z_clip_min', default_value='-2.0'),
        DeclareLaunchArgument('z_clip_max', default_value='2.0'),
        DeclareLaunchArgument('min_points_per_cell', default_value='5'),
        DeclareLaunchArgument('step_threshold_m', default_value='0.15'),
        DeclareLaunchArgument('step_max_threshold_m', default_value='1.0'),
        DeclareLaunchArgument('density_min_pts_per_m3', default_value='40.0'),
        DeclareLaunchArgument('min_points_for_density', default_value='3'),
        Node(
            package='rm_ta',
            executable='traversability_costmap_node',
            name='traversability_costmap_node',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'frame_id': frame_id,
                'resolution': resolution,
                'width_m': width_m,
                'height_m': height_m,
                'origin_x': origin_x,
                'origin_y': origin_y,
                'z_clip_min': z_clip_min,
                'z_clip_max': z_clip_max,
                'min_points_per_cell': min_points_per_cell,
                'step_threshold_m': step_threshold_m,
            }]
        )
    ]) 