cmake_minimum_required(VERSION 3.8)
project(plato2_state_estimator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdr_grasp_msgs REQUIRED)
find_package(plato_utils REQUIRED)

# Include directories
include_directories(include)

# Build core library (uses plato_utils for PID feedback)
add_library(object_state_estimator SHARED
  src/object_state_estimator.cpp
)
ament_target_dependencies(object_state_estimator
  plato_utils
)

# Add executables
add_executable(contact_estimator src/contact_estimator.cpp)
ament_target_dependencies(contact_estimator
  rclcpp
  geometry_msgs
  std_msgs)

add_executable(object_state_estimator_node src/object_state_estimator_node.cpp)
target_link_libraries(object_state_estimator_node
  object_state_estimator
)
ament_target_dependencies(object_state_estimator_node
  rclcpp
  geometry_msgs
  std_msgs
  sdr_grasp_msgs
  plato_utils)

# Install library
install(TARGETS object_state_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  contact_estimator
  object_state_estimator_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()