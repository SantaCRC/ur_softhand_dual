cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_device_msgs)

# Dependency Settings

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Info.msg
        msg/ResourceData.msg
        msg/State.msg
        msg/StateStamped.msg
        msg/DeviceConnectionInfo.msg
        msg/ConnectionState.msg
        srv/GetMeasurements.srv
        srv/InitializeDevice.srv
        srv/SetCommands.srv
        srv/SetPid.srv
        srv/Trigger.srv
        srv/SetControlMode.srv
    DEPENDENCIES
        std_msgs
)

ament_export_dependencies(std_msgs)
ament_package()

# Installation
install(
  DIRECTORY
    msg/
  DESTINATION
    share/${PROJECT_NAME}/msg
)

install(
  DIRECTORY
    srv/
  DESTINATION
    share/${PROJECT_NAME}/srv
)