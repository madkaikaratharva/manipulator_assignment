cmake_minimum_required(VERSION 3.8)
project(manipulator_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(waypoint_test src/waypoint_test.cpp)
ament_target_dependencies(waypoint_test rclcpp trajectory_msgs)

# Add executables
add_executable(plan_and_execute src/plan_and_execute.cpp)
ament_target_dependencies(plan_and_execute rclcpp trajectory_msgs geometry_msgs moveit_ros_planning_interface moveit_ros_planning)

# Add executables
add_executable(waypoint_following src/waypoint_following.cpp)
ament_target_dependencies(waypoint_following rclcpp trajectory_msgs geometry_msgs moveit_ros_planning_interface moveit_ros_planning)


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  waypoint_test
  plan_and_execute
  waypoint_following
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include launch config
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
endif()

ament_package()
