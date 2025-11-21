from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port", default="/dev/ttyACM0")
    baud = LaunchConfiguration("baud", default="115200")
    reopen_interval_ms = LaunchConfiguration("reopen_interval_ms", default="500")
    read_loop_hz = LaunchConfiguration("read_loop_hz", default="200.0")
    tx_hz = LaunchConfiguration("tx_hz", default="100.0")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=port),
            DeclareLaunchArgument("baud", default_value=baud),
            DeclareLaunchArgument(
                "reopen_interval_ms", default_value=reopen_interval_ms
            ),
            DeclareLaunchArgument("read_loop_hz", default_value=read_loop_hz),
            DeclareLaunchArgument("tx_hz", default_value=tx_hz),
            # 直接使用本包串口节点，无需 serial_driver
            Node(
                package="rm_comm_ros2",
                executable="serial_rw_node",
                name="serial_rw_node",
                parameters=[
                    {
                        "port": port,
                        "baud": baud,
                        "reopen_interval_ms": reopen_interval_ms,
                        "read_loop_hz": read_loop_hz,
                    }
                ],
            ),
            Node(
                package="rm_comm_ros2",
                executable="handler_node",
                name="handler_node",
                parameters=[{"tx_hz": tx_hz}],
            ),
        ]
    )
