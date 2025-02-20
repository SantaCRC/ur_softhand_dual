cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_device_driver LANGUAGES CXX)

# Dependency Settings
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qb_device_msgs REQUIRED)

add_subdirectory(api/qbdevice-api-7.x.x)

include_directories(include
)

# Build Settings
add_executable(qb_device_communication_handler
  src/main.cpp
  src/qb_device_communication_handler.cpp
)

target_include_directories(qb_device_communication_handler
  PRIVATE
    ${Serial_SOURCE_DIR}/include/serial
)

target_link_libraries(qb_device_communication_handler
  PRIVATE
    Serial
    qbrobotics_driver
)

ament_target_dependencies(qb_device_communication_handler
  PUBLIC
    "qb_device_msgs"
    "rclcpp"
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp qb_device_msgs)
ament_package()

# Installation
install(
  TARGETS qb_device_communication_handler
  RUNTIME DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    share/${PROJECT_NAME}/launch
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(qb_device_communication_handler
    PUBLIC
      "-std=c++17"
  )
else ()
  set_property(
    TARGET
    qb_device_communication_handler
    PROPERTY CXX_STANDARD 17
  )

  set_property(
    TARGET
      qb_device_communication_handler
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()