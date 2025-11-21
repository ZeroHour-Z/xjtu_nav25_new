from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    target_twist_map_topic = LaunchConfiguration('target_twist_map_topic')
    params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('global_velocity_controller'),
        'config',
        'controller_params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('target_twist_map_topic', default_value='/target_twist_map'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='global_velocity_controller',
            executable='global_velocity_controller_node',
            name='global_velocity_controller',
            output='screen',
            parameters=[
                ParameterFile(params_file, allow_substs=True),
                {
                    'map_frame': map_frame,
                    'base_frame': base_frame,
                    'cmd_vel_topic': cmd_vel_topic,
                    'odom_topic': odom_topic,
                    'target_twist_map_topic': target_twist_map_topic,
                    'global_costmap_topic': '/global_costmap/costmap',
                    'simulate': False,
                    'escape_target_cost_threshold': 0,
                    'escape_enter_cost_threshold': 1,
                    'escape_lethal_threshold': 100,
                    'escape_treat_unknown_as_lethal': False,
                    'escape_speed': 0.75,
                    'escape_goal_tolerance': 0.05,
                    'escape_max_radius_cells': 300,
                }
            ]
        )
    ])
