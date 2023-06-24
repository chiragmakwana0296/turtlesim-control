cmake_minimum_required(VERSION 3.8)
project(turtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_interface REQUIRED)


include_directories(
  include
)

add_executable(pid_pose_controller src/pid_pose_controller.cpp)
ament_target_dependencies(pid_pose_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  turtlesim
  turtle_interface
)
set_target_properties(pid_pose_controller PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

add_executable(rotate_circle src/rotate_circle.cpp)
ament_target_dependencies(rotate_circle
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  turtlesim
  turtle_interface
)
set_target_properties(rotate_circle PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

add_executable(chase_turtle src/chase_turtle.cpp)
ament_target_dependencies(chase_turtle
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  turtlesim
  turtle_interface
)
set_target_properties(chase_turtle PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

add_executable(follow_grid src/follow_grid.cpp)
ament_target_dependencies(follow_grid
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  turtlesim
  turtle_interface
)
set_target_properties(follow_grid PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# Install executable
install(TARGETS pid_pose_controller rotate_circle chase_turtle follow_grid
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY config
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
