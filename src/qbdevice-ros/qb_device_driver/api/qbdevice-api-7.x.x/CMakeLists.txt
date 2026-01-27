cmake_minimum_required(VERSION 3.5.1)
project(qbrobotics_device_API VERSION 1.1.7 LANGUAGES CXX)
message("-- [ ${PROJECT_NAME}] start compiling")

# Add dependencies in the order you want them to build.
add_subdirectory(serial)
add_subdirectory(qbrobotics-driver)
