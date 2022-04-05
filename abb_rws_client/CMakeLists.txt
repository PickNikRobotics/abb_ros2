cmake_minimum_required(VERSION 3.8)
project(abb_rws_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
      -Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    abb_egm_rws_managers
    abb_egm_msgs
    abb_robot_msgs
    abb_rapid_msgs
    abb_rapid_sm_addin_msgs
    rclcpp
    sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

add_executable(rws_client
  src/rws_client_node.cpp
  src/rws_client.cpp
  src/rws_service_provider_ros.cpp
  src/rws_state_publisher_ros.cpp
  src/utilities.cpp
  src/mapping.cpp
)
ament_target_dependencies(rws_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(rws_client  abb_egm_rws_managers::abb_egm_rws_managers)
target_include_directories(
  rws_client
  PRIVATE
  include
)

#############
## Install ##
#############

install(
  TARGETS rws_client
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
