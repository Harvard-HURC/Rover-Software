cmake_minimum_required(VERSION 3.8)
project(rover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# The rover's numbers, its description, the tread tyre and the RViz view
# (README.md).
install(DIRECTORY config meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # xacro and check_urdf on the description, its ros2_control block, the
  # wheel mesh and the view launch file (unittest, as rover_control's and
  # rover_sim's Python tests).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_description
    COMMAND "${Python3_EXECUTABLE}" -m unittest -v test_description
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
endif()

ament_package()
