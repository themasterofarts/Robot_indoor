cmake_minimum_required(VERSION 3.8)
project(indoor_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install config + launch
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python package (indoor_navigation/)
ament_python_install_package(${PROJECT_NAME})

# Install Python node(s) as executables
install(PROGRAMS
  indoor_navigation/frontier_explorer.py
  indoor_navigation/actor_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
