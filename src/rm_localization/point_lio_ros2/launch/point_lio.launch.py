import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_ros2_cfg_dir = LaunchConfiguration("point_lio_ros2_cfg_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")

    point_lio_ros2_dir = get_package_share_directory("point_lio_ros2")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulated time (clock) for all nodes",
    )

    declare_point_lio_ros2_cfg_dir = DeclareLaunchArgument(
        "point_lio_ros2_cfg_dir",
        default_value=PathJoinSubstitution(
            [point_lio_ros2_dir, "config", "mid360.yaml"]
        ),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_ros2_cfg_dir, {"use_sim_time": use_sim_time}],
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_ros2_dir, "rviz_cfg", "loam_livox.rviz"]),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_point_lio_ros2_cfg_dir)
    ld.add_action(start_point_lio_ros2_node)
    ld.add_action(start_rviz_node)

    return ld
