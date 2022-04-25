cmake_minimum_required(VERSION 3.10.2)
project(abb_resources)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})
