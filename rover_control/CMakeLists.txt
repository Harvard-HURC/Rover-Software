cmake_minimum_required(VERSION 3.20)
project(rover_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# MSVC (the Windows build) gets /W4: its -Wall would be /Wall, every warning of every header. It exports every
# symbol, so that a shared library has an import library.
if(MSVC)
  set(warnings /W4)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else()
  set(warnings -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# tank_kinematics: the rover's skid-steer kinematics, its geometry from rover_description's rover.yaml
# (include/rover_control/tank_kinematics.hpp).
add_library(tank_kinematics STATIC src/tank_kinematics.cpp)
target_include_directories(tank_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(tank_kinematics PUBLIC yaml-cpp::yaml-cpp)
set_target_properties(tank_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(tank_kinematics PRIVATE ${warnings})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

include(CTest)
if(BUILD_TESTING)
  # GoogleTest: the tank kinematics, and controllers.yaml against rover_description's rover.yaml.
  find_package(GTest REQUIRED)
  find_package(rover_description REQUIRED)
  add_executable(rover_control_tests
    test/test_tank_kinematics.cpp
    test/test_controllers_yaml.cpp
  )
  target_compile_definitions(rover_control_tests PRIVATE
    ROVER_YAML="${rover_description_DIR}/../config/rover.yaml"
    CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/controllers.yaml")
  target_link_libraries(rover_control_tests PRIVATE tank_kinematics GTest::gtest_main)
  target_compile_options(rover_control_tests PRIVATE ${warnings})
  include(GoogleTest)
  gtest_discover_tests(rover_control_tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(tank_kinematics)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(yaml-cpp)
ament_package()
