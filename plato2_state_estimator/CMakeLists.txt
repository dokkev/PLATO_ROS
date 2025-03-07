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

# Include directories
include_directories(include)

# Add executable
add_executable(contact_estimator src/contact_estimator.cpp)
ament_target_dependencies(contact_estimator
  rclcpp
  geometry_msgs
  std_msgs)

# Install executables
install(TARGETS
  contact_estimator
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files (if any)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()