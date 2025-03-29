cmake_minimum_required(VERSION 3.8)
project(px4_ci_aws)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install launch and config folders
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# install Python node as executable
install(PROGRAMS
  src/px4_arm_and_offboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
