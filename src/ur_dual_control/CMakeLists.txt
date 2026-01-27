cmake_minimum_required(VERSION 3.10)
project(ur_dual_control)

find_package(ament_cmake REQUIRED)

# Instalar los directorios de configuración
install(
  DIRECTORY launch config urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

# Instalar scripts de Python como ejecutables
install(
  PROGRAMS scripts/merge.py  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
