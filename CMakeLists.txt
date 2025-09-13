cmake_minimum_required(VERSION 3.5)
project(lucia_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(YARP REQUIRED)
find_package(robot_localization REQUIRED)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  robot_localization
)

include_directories(
  include
)

add_executable(lucia_controller_ekf_node src/controller_ekf.cpp)
ament_target_dependencies(lucia_controller_ekf_node ${DEPENDENCIES})
target_link_libraries(lucia_controller_ekf_node
  YARP::YARP_os
  YARP::YARP_init
)

add_executable(lucia_controller_node src/controller.cpp)
ament_target_dependencies(lucia_controller_node ${DEPENDENCIES})
target_link_libraries(lucia_controller_node
  YARP::YARP_os
  YARP::YARP_init
)

install(TARGETS
  lucia_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()