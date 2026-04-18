cmake_minimum_required(VERSION 3.5)
project(lucia_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(YARP REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(lucia_controller_node src/lucia_controller.cpp)
ament_target_dependencies(lucia_controller_node rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(lucia_controller_node
  YARP::YARP_os
  YARP::YARP_init
)

add_executable(lucia_minimal_controller_node src/minimal_controller.cpp)
ament_target_dependencies(lucia_minimal_controller_node rclcpp geometry_msgs nav_msgs)
target_link_libraries(lucia_minimal_controller_node
  YARP::YARP_os
  YARP::YARP_init
)

add_executable(lucia_trajectory_drawer_node src/trajectory_drawer.cpp)
ament_target_dependencies(lucia_trajectory_drawer_node rclcpp geometry_msgs nav_msgs)

add_executable(lucia_velocity_smoother_node src/velocity_smoother.cpp)
ament_target_dependencies(lucia_velocity_smoother_node rclcpp geometry_msgs nav_msgs message_filters)

add_executable(joy_to_cmdvel_node src/joy_to_cmdvel.cpp)
ament_target_dependencies(joy_to_cmdvel_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  lucia_controller_node
  lucia_minimal_controller_node
  lucia_trajectory_drawer_node
  lucia_velocity_smoother_node
  joy_to_cmdvel_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()