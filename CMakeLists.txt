cmake_minimum_required(VERSION 3.16)
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
find_package(std_msgs REQUIRED)
find_package(YARP REQUIRED COMPONENTS os)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(lucia_controller hardware/controller.cpp)
ament_target_dependencies(lucia_controller 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# YARPのターゲットをリンク
target_link_libraries(lucia_controller 
  YARP::YARP_os
  YARP::YARP_init
)

install(TARGETS
  lucia_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()