import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("serial_protocol")
    default_mission_file = os.path.join(
        package_path, "config", "mission_example.yaml")

    mission_file = LaunchConfiguration("mission_file")
    auto_start = LaunchConfiguration("auto_start")

    mission_node = Node(
        package="serial_protocol",
        executable="mission_node.py",
        name="mission_node",
        output="screen",
        parameters=[{
            "mission_file": mission_file,
            "auto_start": auto_start,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("mission_file", default_value=default_mission_file),
        DeclareLaunchArgument("auto_start", default_value="true"),
        mission_node,
    ])
