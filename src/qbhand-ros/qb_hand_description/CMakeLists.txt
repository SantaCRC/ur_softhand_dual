cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_hand_description LANGUAGES CXX)
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

install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    meshes/
  DESTINATION
    share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY
    rviz/
  DESTINATION
    share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    urdf/
  DESTINATION
    share/${PROJECT_NAME}/urdf
)
