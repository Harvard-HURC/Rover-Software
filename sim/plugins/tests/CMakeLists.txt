# C++ unit tests of the plugins' parts that need no running Gazebo (sim/CMakeLists.txt adds this directory):
# the drivetrain's motor, dig-in, friction, hub-force and dust rules (rover_drivetrain.hh), the ground map
# reader (terrain_ground.hh) and the ruts' layer and geometry (rover_tracks.hh).
# Run: ctest --test-dir <build directory> --output-on-failure
foreach(name test_rover_drivetrain test_terrain_ground test_rover_tracks)
  add_executable(${name} ${name}.cpp)
  target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(${name} PRIVATE gz-sim8::gz-sim8 gz-common5::geospatial)
  target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
endforeach()
