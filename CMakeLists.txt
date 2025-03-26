cmake_minimum_required(VERSION 3.16)
project(lucia_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament packages and libraries for ament and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(YARP REQUIRED COMPONENTS os)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
#foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#  find_package(${Dependency} REQUIRED)
#endforeach()

include_directories(
  include
)

# ライブラリのビルド
add_library(${PROJECT_NAME}_lib SHARED
  "src/controller.cpp"
)

# find dependencies
set(DEPENDENCIES
  #hardware_interface
  #pluginlib
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

set(CMAKE_BUILD_TYPE Debug)

target_compile_features(lucia_controller_lib PUBLIC cxx_std_17)
target_include_directories(lucia_controller_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include/lucia_controller>
)

ament_target_dependencies(
  lucia_controller_lib PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(lucia_controller_lib PUBLIC
  YARP::YARP_os
  YARP::YARP_init
)

# 実行可能ファイルのビルド
add_executable(lucia_controller_node
  src/controller.cpp
)

target_compile_features(lucia_controller_node PUBLIC cxx_std_17)
target_include_directories(lucia_controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include/lucia_controller>
)

ament_target_dependencies(
  lucia_controller_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(lucia_controller_node
  lucia_controller_lib
  YARP::YARP_os
  YARP::YARP_init
)

install(TARGETS lucia_controller_node lucia_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
