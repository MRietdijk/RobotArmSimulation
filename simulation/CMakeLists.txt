cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

INCLUDE_DIRECTORIES(
  include/simulation
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_arm_interface REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

add_executable(cup_publisher src/CSP.cpp src/CupStatePublisher.cpp)
ament_target_dependencies(cup_publisher rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(robot_arm_publisher src/RAJP.cpp src/RobotArmJointPublisher.cpp src/Parser.cpp)
ament_target_dependencies(robot_arm_publisher rclcpp std_msgs urdf sensor_msgs robot_arm_interface)

add_executable(cup_listener src/PUCL.cpp src/PickedUpCupListener.cpp)
ament_target_dependencies(cup_listener rclcpp std_msgs geometry_msgs tf2_ros)

# TODO: Remove test executable
# add_executable(parser_test src/Main.cpp src/Parser.cpp)

install(TARGETS
  cup_publisher
  robot_arm_publisher
  cup_listener
  DESTINATION lib/${PROJECT_NAME}
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
endif()

ament_package()
