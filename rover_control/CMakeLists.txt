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
# The ROS packages' exported link lines name their messages' Python extension libraries (*__rosidl_generator_py),
# whose Python symbols only a process with libpython has. ros2_control_node, which loads the stub, has none, and on
# macOS a library that is linked is loaded and fails at its first Python symbol. So the linker keeps only the
# libraries a target uses, as rover_sim's plugins do and as conda-forge builds the ROS packages themselves.
if(APPLE)
  set(used_libraries_only "LINKER:-dead_strip_dylibs")
elseif(NOT MSVC)
  set(used_libraries_only "LINKER:--as-needed")
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

# tank_kinematics: the rover's skid-steer kinematics, its geometry from rover_description's rover.yaml
# (include/rover_control/tank_kinematics.hpp).
add_library(tank_kinematics STATIC src/tank_kinematics.cpp)
target_include_directories(tank_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(tank_kinematics PUBLIC yaml-cpp::yaml-cpp)
set_target_properties(tank_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(tank_kinematics PRIVATE ${warnings})

# rover_hardware_stub: the real rover's ros2_control hardware until its motor controllers are chosen
# (pluginlib: rover_control/RoverHardwareStub, rover_hardware_stub.xml).
add_library(rover_hardware_stub SHARED src/rover_hardware_stub.cpp)
target_include_directories(rover_hardware_stub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(rover_hardware_stub PUBLIC hardware_interface::hardware_interface pluginlib::pluginlib
                      rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)
target_compile_options(rover_hardware_stub PRIVATE ${warnings})
target_link_options(rover_hardware_stub PRIVATE ${used_libraries_only})
pluginlib_export_plugin_description_file(hardware_interface rover_hardware_stub.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS tank_kinematics rover_hardware_stub EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# The controllers and the launch files: rover.launch.py, and controllers.launch.py, which rover_sim's sim.launch.py
# includes too (README.md).
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  # GoogleTest: the tank kinematics, controllers.yaml against rover_description's rover.yaml, the hardware stub.
  find_package(GTest REQUIRED)
  find_package(rover_description REQUIRED)
  add_executable(rover_control_tests
    test/test_tank_kinematics.cpp
    test/test_controllers_yaml.cpp
    test/test_rover_hardware_stub.cpp
  )
  target_compile_definitions(rover_control_tests PRIVATE
    ROVER_YAML="${rover_description_DIR}/../config/rover.yaml"
    CONTROLLERS_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/controllers.yaml")
  target_link_libraries(rover_control_tests PRIVATE tank_kinematics rover_hardware_stub GTest::gtest_main)
  target_compile_options(rover_control_tests PRIVATE ${warnings})
  target_link_options(rover_control_tests PRIVATE ${used_libraries_only})
  include(GoogleTest)
  gtest_discover_tests(rover_control_tests)
  # rover.launch.py with the stub (unittest, as rover_description runs its own): this package's install first on
  # AMENT_PREFIX_PATH, so that ros2 launch and pluginlib find it.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_rover_launch COMMAND "${Python3_EXECUTABLE}" -m unittest -v test_rover_launch
           WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test")
  # The prefixes go before the AMENT_PREFIX_PATH the test runs in, at test time: $ENV{} here would bake in what the
  # configure step saw, which on CI's first Ubuntu run (2026-10-11) lacked rover_description.
  get_filename_component(rover_description_PREFIX "${rover_description_DIR}/../../.." ABSOLUTE)
  set_tests_properties(test_rover_launch PROPERTIES TIMEOUT 180
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
    ENVIRONMENT_MODIFICATION "AMENT_PREFIX_PATH=path_list_prepend:${rover_description_PREFIX};AMENT_PREFIX_PATH=path_list_prepend:${CMAKE_INSTALL_PREFIX}")
endif()

ament_export_include_directories(include)
ament_export_libraries(tank_kinematics rover_hardware_stub)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml-cpp)
ament_package()
