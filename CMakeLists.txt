cmake_minimum_required(VERSION 3.16)
project(lucia_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  tf2  
  tf2_ros
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(YARP REQUIRED COMPONENTS os)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPITE
add_library(
  lucia_controller
  SHARED
  hardware/controller.cpp
  #hardware/hardwareHandler.cpp
)

target_compile_features(lucia_controller PUBLIC cxx_std_17)
target_include_directories(lucia_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/lucia_controller>
)

#add_executable(lucia_controller hardware/controller.cpp)
ament_target_dependencies(
  lucia_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS} 
)

target_link_libraries(lucia_controller PUBLIC
  #serial
  YARP::YARP_os
  YARP::YARP_init
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROBOT_CONTROLLER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lucia_controller.xml)

#install(
#  DIRECTORY hardware/include
#  DESTINATION include/robot_controller
#)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/robot_controller
)
install(
  DIRECTORY bringup/launch bringup/config bringup/rviz bringup/map
  DESTINATION share/robot_controller
)
install(TARGETS lucia_controller
  EXPORT export_lucia_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_lucia_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()