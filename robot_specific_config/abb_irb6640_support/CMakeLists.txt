cmake_minimum_required(VERSION 3.8)
project(abb_irb6640_support)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_irb6640_185_280_urdf_xacro test/irb6640_185_280_test_urdf_xacro.py)
endif()

ament_package()
