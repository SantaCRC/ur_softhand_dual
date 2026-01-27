cmake_minimum_required(VERSION 3.16)
project(qb_hand_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
  qb_device_ros2_control
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(qb_device_ros2_control REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  qb_hand_ros2_control
  SHARED
  hardware/qb_hand_ros2_control.cpp
  hardware/qb_hand_2m_ros2_control.cpp
)

target_link_libraries(qb_hand_ros2_control
  PUBLIC
  qb_device_ros2_control::qb_device_ros2_control
)

target_compile_features(qb_hand_ros2_control PUBLIC cxx_std_17)
target_include_directories(qb_hand_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/qb_hand_ros2_control>
)
ament_target_dependencies(
  qb_hand_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "QB_HAND_ROS2_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface qb_hand_ros2_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/qb_hand_ros2_control
)
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/qb_hand_ros2_control
# )
# Installation
install(
  DIRECTORY
    launch/
  DESTINATION
    share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)

install(TARGETS qb_hand_ros2_control
  EXPORT export_qb_hand_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_qb_hand_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
