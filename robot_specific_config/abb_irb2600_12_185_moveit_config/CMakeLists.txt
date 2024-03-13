cmake_minimum_required(VERSION 3.10.2)
project(abb_irb2600_12_185_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)
