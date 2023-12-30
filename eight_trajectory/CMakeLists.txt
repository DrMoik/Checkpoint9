cmake_minimum_required(VERSION 3.8)
project(eight_trajectory)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Added this line

# Add executable
add_executable(eight_trajectory src/eight_trajectory.cpp)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Ament target dependencies
ament_target_dependencies(eight_trajectory
  rclcpp
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs # Added this line
)

# Install the executable
install(TARGETS eight_trajectory
  DESTINATION lib/${PROJECT_NAME})

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
