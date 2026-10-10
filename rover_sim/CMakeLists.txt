cmake_minimum_required(VERSION 3.20)
project(rover_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# The installed plugins keep the paths of the libraries they link (the pixi environment's).
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# On Windows the simulation runs in WSL2 (README, Platforms): a native build
# configures this package to nothing, so the README's `colcon build` builds the
# rover's other packages there.
if(WIN32)
  message(STATUS "rover_sim: nothing to build on Windows (the simulation runs in WSL2)")
  ament_package()
  return()
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS geospatial)

# One Gazebo system plugin per plugins/<name>.cpp, named in CamelCase:
# rocker_differential.cpp -> libRockerDifferential, which a model loads as
# <plugin filename="RockerDifferential">. Gazebo searches
# GZ_SIM_SYSTEM_PLUGIN_PATH, which gzenv.py points at the build directory
# (the workspace's build/rover_sim, where colcon builds this package; any
# `cmake -B` directory works) and a sourced install/ at lib/rover_sim
# (env-hooks/rover_sim.dsv.in). Every plugin may use gz-rendering (camera
# projection) and gz-common's ImageHeightmap (plugins/terrain_heightmap.hh);
# the linker keeps only what a plugin uses.
set(plugin_targets "")
file(GLOB plugin_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp")
foreach(source IN LISTS plugin_sources)
  get_filename_component(stem "${source}" NAME_WE)
  string(REPLACE "_" ";" words "${stem}")
  set(target "")
  foreach(word IN LISTS words)
    string(SUBSTRING "${word}" 0 1 first)
    string(SUBSTRING "${word}" 1 -1 rest)
    string(TOUPPER "${first}" first)
    string(APPEND target "${first}${rest}")
  endforeach()
  add_library(${target} SHARED "${source}")
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/plugins")
  target_link_libraries(${target} PRIVATE gz-sim8::gz-sim8 gz-plugin2::register gz-rendering8::gz-rendering8
                        gz-common5::geospatial)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  if(APPLE)
    target_link_options(${target} PRIVATE "LINKER:-dead_strip_dylibs")
  else()
    target_link_options(${target} PRIVATE "LINKER:--as-needed")
  endif()
  list(APPEND plugin_targets ${target})
endforeach()
install(TARGETS ${plugin_targets} LIBRARY DESTINATION lib/${PROJECT_NAME})

# RoverSimSystem: the simulated rover's ros2_control hardware (ros2_control/), which gz_ros2_control loads inside
# Gazebo through pluginlib (rover_sim_system.xml): each wheel's velocity command into the drivetrain's motor model
# (plugins/rover_components.hh), the joints' state and the motor currents back. It takes only gz_ros2_control's
# interface header: gz_ros2_control's library is the Gazebo system that loads this one.
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
set(gz_ros2_control_include "${gz_ros2_control_DIR}/../../../include/gz_ros2_control")
if(TARGET gz_ros2_control::gz_ros2_control-system)
  get_target_property(gz_ros2_control_include gz_ros2_control::gz_ros2_control-system INTERFACE_INCLUDE_DIRECTORIES)
endif()
add_library(rover_sim_system SHARED ros2_control/rover_sim_system.cpp)
target_include_directories(rover_sim_system PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ros2_control"
                           "${CMAKE_CURRENT_SOURCE_DIR}/plugins" ${gz_ros2_control_include})
target_link_libraries(rover_sim_system PUBLIC gz-sim8::gz-sim8 hardware_interface::hardware_interface
                      pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_options(rover_sim_system PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS rover_sim_system LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(gz_ros2_control rover_sim_system.xml)

# C++ unit tests of the plugins' parts that need no Gazebo running, and of RoverSimSystem against a bare ECM:
# pixi run colcon-test, or ctest --test-dir <build directory>.
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/tests/CMakeLists.txt")
  add_subdirectory(plugins/tests)
endif()
find_package(GTest REQUIRED)
add_executable(test_rover_sim_system ros2_control/test/test_rover_sim_system.cpp)
target_link_libraries(test_rover_sim_system PRIVATE rover_sim_system GTest::gtest)
target_compile_options(test_rover_sim_system PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME test_rover_sim_system COMMAND test_rover_sim_system)

# The ament linters (ament_lint_auto) do not run on this package: its code keeps
# the simulation's own style (120 columns, its docstrings), which flake8, pep257
# and uncrustify with ROS settings reject file by file.

# The launch files, with the wrapper xacro and sim.yaml they read. sim.launch.py runs the simulation from this source
# tree (its generated worlds and models are here, and gzenv.py sets Gazebo's environment from it): the installed copy
# finds the tree through share/rover_sim/source_dir.
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/source_dir" "${CMAKE_CURRENT_SOURCE_DIR}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/source_dir" DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
