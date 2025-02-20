cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_device_bringup LANGUAGES CXX)
find_package(ament_cmake REQUIRED)

# Dependency Settings

ament_package()

# Installation
install(
  DIRECTORY
    launch/
  DESTINATION
    share/${PROJECT_NAME}/launch
)