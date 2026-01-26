import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sr = math.sin(roll * 0.5)
    sp = math.sin(pitch * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _launch_setup(context, *args, **kwargs):
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    use_rviz = LaunchConfiguration("rviz").perform(context)
    use_rviz_condition = IfCondition(LaunchConfiguration("rviz"))

    # Params for static transforms
    M_PI_2 = float(LaunchConfiguration("M_PI_2").perform(context))
    ROLL = float(LaunchConfiguration("ROLL").perform(context))
    R = float(LaunchConfiguration("R").perform(context))
    H = float(LaunchConfiguration("H").perform(context))

    # TF: body -> base_link
    # ROS1 used: x y z yaw pitch roll
    # Here we convert to quaternion with roll=0, pitch=ROLL, yaw=M_PI_2
    qx1, qy1, qz1, qw1 = _rpy_to_quaternion(0.0, ROLL, M_PI_2)
    static_tf_body2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0",
            str(R),
            str(-H),
            str(qx1),
            str(qy1),
            str(qz1),
            str(qw1),
            "body",
            "base_link",
        ],
        remappings=remappings,
        output="screen",
    )

    # TF: camera_init -> map
    qx2, qy2, qz2, qw2 = _rpy_to_quaternion(0.0, 0.0, M_PI_2)
    static_tf_map_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0.15",
            "-0.25",
            str(qx2),
            str(qy2),
            str(qz2),
            str(qw2),
            "camera_init",
            "map",
        ],
        remappings=remappings,
        output="screen",
    )

    # Point-LIO node
    point_lio_ros2_dir = get_package_share_directory("point_lio_ros2")
    point_lio_ros2_cfg = PathJoinSubstitution(
        [point_lio_ros2_dir, "config", "build_map.yaml"]
    ).perform(context)
    point_lio_ros2_params = {
        "use_imu_as_input": False,
        "prop_at_freq_of_imu": False,
        "check_satu": True,
        "init_map_size": 10,
        "point_filter_num": 3,
        "space_down_sample": True,
        "filter_size_surf": 0.1,
        "filter_size_map": 0.2,
        "cube_side_length": 500.0,
        "runtime_pos_log_enable": False,
        "use_sim_time": False,
    }
    point_lio_ros2_params_list = []
    # Append YAML if exists
    if os.path.exists(point_lio_ros2_cfg):
        point_lio_ros2_params_list.append(point_lio_ros2_cfg)
    point_lio_ros2_params_list.append(point_lio_ros2_params)

    point_lio_ros2_node = Node(
        package="point_lio_ros2",
        executable="pointlio_mapping",
        parameters=point_lio_ros2_params_list,
        remappings=remappings,
        output="screen",
    )

    # RViz2 node (best-effort: prefer fast_lio_localization config, else fallback)
    rviz_node = None
    if use_rviz.lower() in ["1", "true", "True", "TRUE"]:
        config_path = None
        try:
            fll_dir = get_package_share_directory("fast_lio_localization")
            candidate = os.path.join(fll_dir, "rviz_cfg", "sentry_build_map.rviz")
            if os.path.exists(candidate):
                config_path = candidate
        except Exception:
            config_path = None
        if config_path is None:
            # fallback to a default rviz under point_lio_ros2 if available
            candidate = os.path.join(point_lio_ros2_dir, "rviz_cfg", "loam_livox.rviz")
            if os.path.exists(candidate):
                config_path = candidate
        # Assemble rviz2 node if we have a config
        if config_path is not None:
            rviz_node = Node(
                condition=use_rviz_condition,
                package="rviz2",
                executable="rviz2",
                name="rviz",
                remappings=remappings,
                arguments=["-d", config_path],
            )

    # Optional include: 2D map build if ROS2 version exists
    include_pointcloud2map = None
    try:
        fll_dir = get_package_share_directory("fast_lio_localization")
        candidate_py = os.path.join(fll_dir, "launch", "Pointcloud2Map.launch.py")
        if os.path.exists(candidate_py):
            include_pointcloud2map = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(candidate_py)
            )
    except Exception:
        include_pointcloud2map = None

    actions = [point_lio_ros2_node, static_tf_body2base, static_tf_map_init]
    if rviz_node is not None:
        actions.append(rviz_node)
    if include_pointcloud2map is not None:
        actions.append(include_pointcloud2map)
    return actions


def generate_launch_description():
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="是否启动 RViz2"
    )
    declare_M_PI_2 = DeclareLaunchArgument(
        "M_PI_2",
        default_value="1.5707963267948966",
        description="常量 π/2",
    )
    declare_ROLL = DeclareLaunchArgument(
        "ROLL",
        default_value="0.2617993877991494",
        description="安装角 ROLL (rad)",
    )
    declare_R = DeclareLaunchArgument(
        "R",
        default_value="0.12848040398218347",
        description="机体系相对于雷达系的 Y 方向偏移 (m)",
    )
    declare_H = DeclareLaunchArgument(
        "H",
        default_value="0.2932452655927712",
        description="机体系相对于雷达系的 Z 方向偏移 (m)",
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)
    ld.add_action(declare_M_PI_2)
    ld.add_action(declare_ROLL)
    ld.add_action(declare_R)
    ld.add_action(declare_H)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
