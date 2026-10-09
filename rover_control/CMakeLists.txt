cmake_minimum_required(VERSION 3.20)
project(rover_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# MSVC (the Windows build) gets /W4: its -Wall would be /Wall, every warning of every header.
if(MSVC)
  set(warnings /W4)
else()
  set(warnings -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# rover_driver: the swerve design's kinematics, superseded by tank drive and kept
# until the ros2_control work replaces it (design spec 2026-10-09, section 9.2).
add_library(rover_driver STATIC
  src/kinematics_2d.cpp
  src/kinematics_3d.cpp
  src/steering.cpp
)
target_include_directories(rover_driver PUBLIC include)
target_link_libraries(rover_driver PUBLIC Eigen3::Eigen)
target_compile_options(rover_driver PRIVATE ${warnings})

add_executable(rover_state tools/rover_state.cpp)
target_link_libraries(rover_state PRIVATE rover_driver)
target_compile_options(rover_state PRIVATE ${warnings})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rover_driver ARCHIVE DESTINATION lib)
install(TARGETS rover_state RUNTIME DESTINATION lib/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(rover_driver_tests
    tests/test_kinematics_2d.cpp
    tests/test_kinematics_3d.cpp
    tests/test_steering.cpp
  )
  target_link_libraries(rover_driver_tests PRIVATE rover_driver GTest::gtest_main)
  target_compile_options(rover_driver_tests PRIVATE ${warnings})
  include(GoogleTest)
  gtest_discover_tests(rover_driver_tests)
  add_test(NAME rover_state_runs COMMAND rover_state --mode 3d --vx 0.5 --wz 0.2 --ql 0.1 --dqr -0.2)
  add_test(NAME rover_state_rejects_bad_flag COMMAND rover_state --bogus 1)
  # strtod accepts nan, inf and out-of-range values (as inf); JSON has neither.
  add_test(NAME rover_state_rejects_nan COMMAND rover_state --vx nan)
  add_test(NAME rover_state_rejects_out_of_range COMMAND rover_state --wz 1e999)
  set_tests_properties(rover_state_rejects_bad_flag rover_state_rejects_nan
                       rover_state_rejects_out_of_range PROPERTIES WILL_FAIL TRUE)
  # A rejection names its reason (the exit code is checked by the tests above).
  add_test(NAME rover_state_explains_rejection COMMAND rover_state --vx abc)
  set_tests_properties(rover_state_explains_rejection PROPERTIES
                       PASS_REGULAR_EXPRESSION "invalid value 'abc' for --vx")
  # viz.py reads the output with json.loads, so it must parse as JSON in both
  # modes, also when a huge but finite command overflows in the kinematics.
  find_package(Python3 COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    function(add_rover_state_json_test name)
      string(JOIN " " args ${ARGN})
      set(pipeline "'$<TARGET_FILE:rover_state>' ${args} | '${Python3_EXECUTABLE}' -m json.tool")
      add_test(NAME ${name} COMMAND sh -c "${pipeline}")
    endfunction()
    add_rover_state_json_test(rover_state_json_3d --mode 3d --vx 0.5 --wz 0.2 --ql 0.1 --dqr -0.2)
    add_rover_state_json_test(rover_state_json_2d --mode 2d --vx 0.5 --vy -0.3 --wz 0.2)
    add_rover_state_json_test(rover_state_json_overflow --mode 3d --vx 1e300 --gx 1e300)
    # viz.py drawing tests, where its matplotlib and numpy are installed.
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import matplotlib, numpy"
                    RESULT_VARIABLE viz_deps_result OUTPUT_QUIET ERROR_QUIET)
    if(viz_deps_result EQUAL 0)
      add_test(NAME viz_py
               COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_viz.py")
      set(viz_env ROVER_STATE_EXE=$<TARGET_FILE:rover_state> MPLBACKEND=Agg
                  PYTHONDONTWRITEBYTECODE=1)
      set_tests_properties(viz_py PROPERTIES ENVIRONMENT "${viz_env}")
    else()
      message(STATUS "matplotlib or numpy missing: skipping the viz_py test")
    endif()
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(rover_driver)
ament_export_dependencies(Eigen3)
ament_package()
