cmake_minimum_required(VERSION 3.5)
project(serial_protocol)

# ????????
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ??????????
include_directories(include)

# ???????????
add_executable(serial_protocol_node
    src/serial_protocol_node.cpp
)
add_executable(error_protect
    src/error_protect.cpp
)
add_executable(ego_tracker_node
    src/ego_tracker_node.cpp
)

# ????????
ament_target_dependencies(serial_protocol_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    std_srvs
)
ament_target_dependencies(error_protect
    rclcpp
    std_srvs
    nav_msgs
)
ament_target_dependencies(ego_tracker_node
    rclcpp
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    tf2
    tf2_geometry_msgs
    visualization_msgs
)

# ???????????
install(TARGETS
    serial_protocol_node
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
    error_protect
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
    ego_tracker_node
    DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
    scripts/mission_node.py
    scripts/task_manager_node.py
    scripts/visual_cmd_mux_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# ????????????????????
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY docs/
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
