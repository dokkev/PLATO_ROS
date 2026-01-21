cmake_minimum_required(VERSION 3.8)
project(posture_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plato2_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create the posture_controller node executable
add_executable(posture_controller_node src/posture_controller_node.cpp)

# Link dependencies
ament_target_dependencies(posture_controller_node
  rclcpp
  plato2_interfaces
)

# Install executable
install(TARGETS posture_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/ DESTINATION include)

ament_package()
