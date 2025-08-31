cmake_minimum_required(VERSION 3.8)
project(leptrino_force_torque)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create shared library for component
add_library(${PROJECT_NAME}_component SHARED
  src/leptrino_force_torque.cpp
  src/rs_comm.cpp
)

# Target dependencies for component
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  geometry_msgs
)

# Register components
rclcpp_components_register_nodes(${PROJECT_NAME}_component "LeptrinoNode")

# Create executable (for standalone execution)
add_executable(${PROJECT_NAME}
  src/leptrino_force_torque.cpp
  src/rs_comm.cpp
)

# Target dependencies for executable
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
)

# Install executables and libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
