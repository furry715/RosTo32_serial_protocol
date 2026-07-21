from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    position_cmd_topic = LaunchConfiguration("position_cmd_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "position_cmd_topic", default_value="/position_cmd"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel"),
        Node(
            package="serial_protocol",
            executable="ego_tracker_node",
            name="ego_tracker_node",
            output="screen",
            parameters=[{
                "odom_topic": odom_topic,
                "position_cmd_topic": position_cmd_topic,
                "cmd_vel_topic": cmd_vel_topic,
            }],
        ),
    ])
