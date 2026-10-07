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
find_package(gz-rendering8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS geospatial)

# One Gazebo system plugin per plugins/<name>.cpp, named in CamelCase:
# rocker_differential.cpp -> libRockerDifferential, which a model loads as
# <plugin filename="RockerDifferential">. Gazebo searches
# GZ_SIM_SYSTEM_PLUGIN_PATH, which sim/gzenv.py points at the build directory
# (sim/build by default; any `cmake -B` directory works). Every plugin may use
# gz-rendering (camera projection) and gz-common's ImageHeightmap
# (plugins/terrain_heightmap.hh); the linker keeps only what a plugin uses.
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
endforeach()

# C++ unit tests of the plugins' parts that need no Gazebo running:
# ctest --test-dir <build directory>.
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/plugins/tests/CMakeLists.txt")
  add_subdirectory(plugins/tests)
endif()
