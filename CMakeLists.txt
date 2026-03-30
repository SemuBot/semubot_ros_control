cmake_minimum_required(VERSION 3.16)
project(semubot_ros_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  controller_interface
  geometry_msgs
  nav_msgs
  realtime_tools
  tf2
  Eigen3
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  semubot_hardware_interface
  SHARED
  src/semubot_hardware_interface.cpp
)

target_compile_features(semubot_hardware_interface PUBLIC cxx_std_17)
target_include_directories(semubot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/omni_robot_control>
)
ament_target_dependencies(
  semubot_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(semubot_hardware_interface PRIVATE "semubot_hardware_interface_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface semubot_hardware_interface_plugin.xml)

## Controller
add_library(
  semubot_velocity_controller
  SHARED
  src/semubot_velocity_controller.cpp
)

target_compile_features(semubot_velocity_controller PUBLIC cxx_std_17)
target_include_directories(semubot_velocity_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/omni_robot_control>
)
ament_target_dependencies(
  semubot_velocity_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(semubot_velocity_controller PRIVATE "semubot_velocity_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface semubot_velocity_controller_plugin.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/omni_robot_control
)

install(
  TARGETS semubot_hardware_interface semubot_velocity_controller
  EXPORT export_omni_robot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES semubot_hardware_interface_plugin.xml semubot_velocity_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


## EXPORTS
ament_export_targets(export_omni_robot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
