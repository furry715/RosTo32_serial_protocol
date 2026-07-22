import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    flight_type = LaunchConfiguration("flight_type")
    rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    planning_horizon = LaunchConfiguration("planning_horizon")
    goal_yaw_mode = LaunchConfiguration("goal_yaw_mode")
    yaw_done_deg = LaunchConfiguration("yaw_done_deg")

    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")

    point_num = LaunchConfiguration("point_num")
    point0_x = LaunchConfiguration("point0_x")
    point0_y = LaunchConfiguration("point0_y")
    point0_z = LaunchConfiguration("point0_z")
    point1_x = LaunchConfiguration("point1_x")
    point1_y = LaunchConfiguration("point1_y")
    point1_z = LaunchConfiguration("point1_z")
    point2_x = LaunchConfiguration("point2_x")
    point2_y = LaunchConfiguration("point2_y")
    point2_z = LaunchConfiguration("point2_z")
    point3_x = LaunchConfiguration("point3_x")
    point3_y = LaunchConfiguration("point3_y")
    point3_z = LaunchConfiguration("point3_z")
    point4_x = LaunchConfiguration("point4_x")
    point4_y = LaunchConfiguration("point4_y")
    point4_z = LaunchConfiguration("point4_z")

    ego_rviz = "/home/f/linxiao_ws/src/rviz.rviz"

    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner_real",
        output="screen",
        remappings=[
            ("odom_world", odom_topic),
            ("grid_map/odom", odom_topic),
            ("grid_map/cloud", cloud_topic),
        ],
        parameters=[{
            "fsm/flight_type": flight_type,
            "fsm/thresh_replan_time": 1.0,
            "fsm/thresh_no_replan_meter": 1.0,
            "fsm/planning_horizon": planning_horizon,
            "fsm/planning_horizen_time": 3.0,
            "fsm/emergency_time": 1.0,
            "fsm/realworld_experiment": True,
            "fsm/fail_safe": True,
            "fsm/waypoint_num": point_num,
            "fsm/waypoint0_x": point0_x,
            "fsm/waypoint0_y": point0_y,
            "fsm/waypoint0_z": point0_z,
            "fsm/waypoint1_x": point1_x,
            "fsm/waypoint1_y": point1_y,
            "fsm/waypoint1_z": point1_z,
            "fsm/waypoint2_x": point2_x,
            "fsm/waypoint2_y": point2_y,
            "fsm/waypoint2_z": point2_z,
            "fsm/waypoint3_x": point3_x,
            "fsm/waypoint3_y": point3_y,
            "fsm/waypoint3_z": point3_z,
            "fsm/waypoint4_x": point4_x,
            "fsm/waypoint4_y": point4_y,
            "fsm/waypoint4_z": point4_z,
            "grid_map/resolution": 0.1,
            "grid_map/map_size_x": map_size_x,
            "grid_map/map_size_y": map_size_y,
            "grid_map/map_size_z": map_size_z,
            "grid_map/local_update_range_x": 5.5,
            "grid_map/local_update_range_y": 5.5,
            "grid_map/local_update_range_z": 4.5,
            "grid_map/self_clearance_x": 0.20,
            "grid_map/self_clearance_y": 0.20,
            "grid_map/self_clearance_z": 0.22,
            "grid_map/obstacles_inflation": 0.05,
            "grid_map/local_map_margin": 10,
            "grid_map/ground_height": -0.01,
            "grid_map/visualization_truncate_height": 3.0,
            "grid_map/pose_type": 1,
            "grid_map/frame_id": "camera_init",
            "manager/max_vel": max_vel,
            "manager/max_acc": max_acc,
            "manager/max_jerk": 4.0,
            "manager/control_points_distance": 0.4,
            "manager/feasibility_tolerance": 0.05,
            "manager/planning_horizon": planning_horizon,
            "manager/use_distinctive_trajs": False,
            "manager/drone_id": -1,
            "optimization/lambda_smooth": 1.0,
            "optimization/lambda_collision": 0.5,
            "optimization/lambda_feasibility": 0.1,
            "optimization/lambda_fitness": 1.0,
            "optimization/dist0": 0.5,
            "optimization/swarm_clearance": 0.5,
            "optimization/max_vel": max_vel,
            "optimization/max_acc": max_acc,
            "bspline/limit_vel": max_vel,
            "bspline/limit_acc": max_acc,
            "bspline/limit_ratio": 1.1,
            "prediction/obj_num": 0,
            "prediction/lambda": 1.0,
            "prediction/predict_rate": 1.0,
        }],
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="traj_server_real",
        output="screen",
        parameters=[{
            "traj_server/time_forward": 1.0,
        }],
    )

    ego_tracker_node = Node(
        package="serial_protocol",
        executable="ego_tracker_node",
        name="ego_tracker_node",
        output="screen",
        parameters=[{
            "odom_topic": odom_topic,
            "position_cmd_topic": "/position_cmd",
            "cmd_vel_topic": cmd_vel_topic,
            "follow_traj_yaw": False,
            "goal_yaw_mode": goal_yaw_mode,
            "yaw_done_deg": yaw_done_deg,
            "dx": 0.0,
            "dy": 0.0,
            "dz": 0.0,
            "px": 1.2,
            "py": 1.2,
            "pz": 1.1,
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="ego_planner_rviz",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("flight_type", default_value="1"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_cfg", default_value=ego_rviz),
        DeclareLaunchArgument("max_vel", default_value="0.5"),
        DeclareLaunchArgument("max_acc", default_value="2.0"),
        DeclareLaunchArgument("planning_horizon", default_value="7.5"),
        DeclareLaunchArgument("goal_yaw_mode", default_value="hold"),
        DeclareLaunchArgument("yaw_done_deg", default_value="5.0"),
        DeclareLaunchArgument("map_size_x", default_value="42.0"),
        DeclareLaunchArgument("map_size_y", default_value="30.0"),
        DeclareLaunchArgument("map_size_z", default_value="5.0"),
        DeclareLaunchArgument("point_num", default_value="1"),
        DeclareLaunchArgument("point0_x", default_value="3.0"),
        DeclareLaunchArgument("point0_y", default_value="0.0"),
        DeclareLaunchArgument("point0_z", default_value="1.0"),
        DeclareLaunchArgument("point1_x", default_value="6.0"),
        DeclareLaunchArgument("point1_y", default_value="0.0"),
        DeclareLaunchArgument("point1_z", default_value="1.0"),
        DeclareLaunchArgument("point2_x", default_value="6.0"),
        DeclareLaunchArgument("point2_y", default_value="2.0"),
        DeclareLaunchArgument("point2_z", default_value="1.0"),
        DeclareLaunchArgument("point3_x", default_value="3.0"),
        DeclareLaunchArgument("point3_y", default_value="2.0"),
        DeclareLaunchArgument("point3_z", default_value="1.0"),
        DeclareLaunchArgument("point4_x", default_value="0.0"),
        DeclareLaunchArgument("point4_y", default_value="0.0"),
        DeclareLaunchArgument("point4_z", default_value="1.0"),
        ego_planner_node,
        traj_server_node,
        ego_tracker_node,
        rviz_node,
    ])
