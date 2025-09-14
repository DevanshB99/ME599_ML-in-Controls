cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/
)

install(DIRECTORY robot/
  DESTINATION share/${PROJECT_NAME}/robot/
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes/
)

ament_package()