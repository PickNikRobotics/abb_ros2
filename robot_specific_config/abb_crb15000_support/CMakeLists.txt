cmake_minimum_required(VERSION 3.10.2)

project(abb_crb15000_support)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})
