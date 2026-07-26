import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def include_launch(
    package_name, launch_file, launch_arguments=None, condition=None,
    launch_dir="launch"
):
    launch_path = os.path.join(
        get_package_share_directory(package_name), launch_dir, launch_file)
    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_path),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
        condition=condition,
        scoped=True,
    )


def generate_launch_description():
    package_path = get_package_share_directory("serial_protocol")
    default_mission_file = os.path.join(
        package_path, "config", "mission_example.yaml")
    default_task_map_file = os.path.join(
        package_path, "config", "task_map.yaml")

    use_livox = LaunchConfiguration("use_livox")
    use_fast_lio = LaunchConfiguration("use_fast_lio")
    use_ekf = LaunchConfiguration("use_ekf")
    use_ego = LaunchConfiguration("use_ego")
    use_mission = LaunchConfiguration("use_mission")
    use_task_manager = LaunchConfiguration("use_task_manager")
    use_serial = LaunchConfiguration("use_serial")
    rviz = LaunchConfiguration("rviz")

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    send_rate_hz = LaunchConfiguration("send_rate_hz")
    vel_timeout = LaunchConfiguration("vel_timeout")

    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    goal_yaw_mode = LaunchConfiguration("goal_yaw_mode")
    yaw_done_deg = LaunchConfiguration("yaw_done_deg")
    mission_file = LaunchConfiguration("mission_file")
    mission_auto_start = LaunchConfiguration("mission_auto_start")
    task_map_file = LaunchConfiguration("task_map_file")

    livox_launch = include_launch(
        "livox_ros_driver2",
        "msg_MID360_launch.py",
        condition=IfCondition(use_livox),
        launch_dir="launch_ROS2",
    )

    fast_lio_launch = include_launch(
        "fast_lio",
        "mapping.launch.py",
        {
            "config_file": fast_lio_config_file,
            "rviz": "false",
        },
        condition=IfCondition(use_fast_lio),
    )

    ekf_launch = include_launch(
        "robot_localization",
        "linxiao_fastlio_ekf.launch.py",
        condition=IfCondition(use_ekf),
    )

    ego_launch = include_launch(
        "serial_protocol",
        "ego_planner_real.launch.py",
        {
            "odom_topic": odom_topic,
            "cloud_topic": cloud_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "rviz": rviz,
            "goal_yaw_mode": goal_yaw_mode,
            "yaw_done_deg": yaw_done_deg,
        },
        condition=IfCondition(use_ego),
        launch_dir=".",
    )

    mission_launch = include_launch(
        "serial_protocol",
        "mission.launch.py",
        {
            "mission_file": mission_file,
            "auto_start": mission_auto_start,
        },
        condition=IfCondition(use_mission),
        launch_dir=".",
    )

    task_manager_node = Node(
        package="serial_protocol",
        executable="task_manager_node.py",
        name="task_manager_node",
        output="screen",
        parameters=[{
            "task_map_file": task_map_file,
        }],
        condition=IfCondition(use_task_manager),
    )

    serial_node = Node(
        package="serial_protocol",
        executable="serial_protocol_node",
        name="serial_protocol_node",
        output="screen",
        parameters=[{
            "port": port,
            "baud": baud,
            "send_rate_hz": send_rate_hz,
            "vel_timeout": vel_timeout,
        }],
        condition=IfCondition(use_serial),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_livox", default_value="true"),
        DeclareLaunchArgument("use_fast_lio", default_value="true"),
        DeclareLaunchArgument("use_ekf", default_value="false"),
        DeclareLaunchArgument("use_ego", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("use_task_manager", default_value="false"),
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("send_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("vel_timeout", default_value="0.2"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("goal_yaw_mode", default_value="hold"),
        DeclareLaunchArgument("yaw_done_deg", default_value="5.0"),
        DeclareLaunchArgument("mission_file", default_value=default_mission_file),
        DeclareLaunchArgument(
            "mission_auto_start",
            default_value=PythonExpression([
                "'false' if '", use_task_manager, "' == 'true' else 'true'"
            ]),
        ),
        DeclareLaunchArgument("task_map_file", default_value=default_task_map_file),

        livox_launch,
        TimerAction(period=2.0, actions=[fast_lio_launch]),
        TimerAction(period=4.0, actions=[ekf_launch]),
        TimerAction(period=6.0, actions=[ego_launch]),
        TimerAction(period=8.0, actions=[serial_node]),
        TimerAction(period=9.0, actions=[mission_launch]),
        TimerAction(period=9.5, actions=[task_manager_node]),
    ])
