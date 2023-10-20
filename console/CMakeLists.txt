cmake_minimum_required(VERSION 3.8)
project(console)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

INCLUDE_DIRECTORIES(
  include/console
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_arm_interface REQUIRED)

add_executable(console src/main.cpp src/Console.cpp src/Parser.cpp)
ament_target_dependencies(console rclcpp robot_arm_interface)

add_executable(sequence src/Sequence.cpp)
ament_target_dependencies(sequence rclcpp robot_arm_interface)

install(TARGETS
  console
  sequence
  DESTINATION lib/${PROJECT_NAME})



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
endif()

ament_package()
