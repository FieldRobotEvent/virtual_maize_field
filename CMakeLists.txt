cmake_minimum_required(VERSION 3.5)
project(virtual_maize_field)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.sh")

ament_python_install_package(${PROJECT_NAME})

#============================================================================
# Nodes

install(
  PROGRAMS virtual_maize_field/generate_world.py 
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_world
)

#============================================================================
# Resources
install(
  DIRECTORY
    config 
    models 
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
