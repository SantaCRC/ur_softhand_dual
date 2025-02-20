cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(qbrobotics_driver VERSION 7.2.8 LANGUAGES CXX)
get_filename_component(API_PATH ../ ABSOLUTE)
if(EXISTS "${API_PATH}/serial/CMakeLists.txt")
        message("-- [ ${PROJECT_NAME}] Serial folder found")
endif()
SET(CMAKE_PREFIX_PATH ${API_PATH}/serial)
message("-- [ ${PROJECT_NAME}] start compiling")
find_package(Serial REQUIRED)
add_library(qbrobotics_driver 
  STATIC # [SHARED | STATIC]
  libs/research/src/qbrobotics_research_api.cpp
  libs/research/src/qbrobotics_research_api_wrapper.cpp
  libs/research/src/qbsofthand_research_api.cpp
  libs/research/src/qbmove_research_api.cpp
  libs/research/src/qbsofthand2_research_api.cpp
  libs/research/src/qbsoftclaw_api.cpp
)
target_include_directories(qbrobotics_driver PUBLIC
  $<BUILD_INTERFACE:${qbrobotics_driver_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${qbrobotics_driver_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "$<BUILD_INTERFACE:${Serial_SOURCE_DIR}/include>"
)
target_include_directories(qbrobotics_driver 
  PUBLIC  #TODO: make PRIVATE?
    "$<BUILD_INTERFACE:${qbrobotics_driver_BINARY_DIR}/libs/research/include>"
    "$<BUILD_INTERFACE:${qbrobotics_driver_SOURCE_DIR}/libs/research/include>"
  PRIVATE
    "$<BUILD_INTERFACE:${qbrobotics_driver_SOURCE_DIR}/libs/research/src>"
)
target_link_libraries(qbrobotics_driver PRIVATE
  Serial::Serial
)
set_target_properties(qbrobotics_driver PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS NO
)
add_library(qbrobotics_driver::qbrobotics_driver ALIAS qbrobotics_driver)

install(DIRECTORY libs/research/include
  DESTINATION include
)
install(TARGETS qbrobotics_driver EXPORT qbrobotics_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT qbrobotics_driverTargets
  FILE qbrobotics_driverTargets.cmake
  NAMESPACE qbrobotics_driver::
  DESTINATION lib/cmake/qbrobotics_driver
)

include(CMakePackageConfigHelpers)
configure_package_config_file(qbrobotics_driverConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/qbrobotics_driverConfig.cmake
  INSTALL_DESTINATION lib/cmake/qbrobotics_driver
)
write_basic_package_version_file(qbrobotics_driverConfigVersion.cmake
  VERSION ${qbrobotics_driver_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/qbrobotics_driverConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/qbrobotics_driverConfigVersion.cmake
  DESTINATION lib/cmake/qbrobotics_driver
)

# Tests
set(TESTS false) # choose if enabling tests or not
if(TESTS)
  enable_testing()
  find_package(GTest REQUIRED CONFIG)

  add_executable(qbrobotics_research_api_communication
    tests/qbrobotics_research_api_communication.cpp
  )
  target_link_libraries(qbrobotics_research_api_communication PRIVATE
    qbrobotics_driver::qbrobotics_driver
    Serial::Serial
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
  )
  set_target_properties(qbrobotics_research_api_communication PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS NO
  )
  add_test(NAME qbrobotics_research_api_communication_tests
    COMMAND qbrobotics_research_api_communication
  )

  add_executable(qbrobotics_research_api_wrapper_communication
    tests/qbrobotics_research_api_wrapper_communication.cpp
  )
  target_link_libraries(qbrobotics_research_api_wrapper_communication PRIVATE
    qbrobotics_driver::qbrobotics_driver
    Serial::Serial
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
  )
  set_target_properties(qbrobotics_research_api_wrapper_communication PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS NO
  )
  add_test(NAME qbrobotics_research_api_wrapper_communication_tests
    COMMAND qbrobotics_research_api_wrapper_communication
  )
endif()
