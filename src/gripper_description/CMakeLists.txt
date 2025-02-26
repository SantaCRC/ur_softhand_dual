cmake_minimum_required(VERSION 3.10)
project(onrobot_gripper)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
