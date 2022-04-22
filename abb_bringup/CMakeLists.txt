cmake_minimum_required(VERSION 3.5)
project(abb_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  # GTests
  ament_add_gtest_executable(test_command_topics test/test_command_topics.cpp)
  ament_target_dependencies(test_command_topics rclcpp trajectory_msgs)
  add_ros_test(test/launch/test_command_topics.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_export_dependencies(rclcpp)

ament_package()
