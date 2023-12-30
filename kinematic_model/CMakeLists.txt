cmake_minimum_required(VERSION 3.8)
project(kinematic_model)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(kinematic_model src/kinematic_model.cpp)



# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# Ament target dependencies
ament_target_dependencies(kinematic_model
  rclcpp
  std_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS kinematic_model
  DESTINATION lib/${PROJECT_NAME})

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
