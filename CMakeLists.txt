cmake_minimum_required(VERSION 3.5)
project(serial_protocol)

# ????????
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

# ??????????
include_directories(include)

# ???????????
add_executable(serial_protocol_node
    src/serial_protocol_node.cpp
)
add_executable(error_protect
    src/error_protect.cpp
)

# ????????
ament_target_dependencies(serial_protocol_node
    rclcpp
    geometry_msgs
    std_msgs
    std_srvs
)
ament_target_dependencies(error_protect
    rclcpp
    std_srvs
    nav_msgs
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

# ????????????????????
install(DIRECTORY include/
    DESTINATION include
)

ament_package()