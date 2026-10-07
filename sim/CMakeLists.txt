cmake_minimum_required(VERSION 3.20)
project(rover_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Models load it as <plugin filename="RockerDifferential">; Gazebo searches
# GZ_SIM_SYSTEM_PLUGIN_PATH, which sim/run.sh and the tests point here.
add_library(RockerDifferential SHARED plugins/rocker_differential.cpp)
target_link_libraries(RockerDifferential PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(RockerDifferential PRIVATE -Wall -Wextra -Wpedantic)

# Throttled joint states and key-press events for the URC lander (referee input).
add_library(JointMonitor SHARED plugins/joint_monitor.cpp)
target_link_libraries(JointMonitor PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(JointMonitor PRIVATE -Wall -Wextra -Wpedantic)

# The driver station's cameras (sim/station): flies the chase_camera model
# after the rover and keeps eye_camera at the rover's camera pivot.
add_library(ChaseCamera SHARED plugins/chase_camera.cpp)
target_link_libraries(ChaseCamera PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(ChaseCamera PRIVATE -Wall -Wextra -Wpedantic)
