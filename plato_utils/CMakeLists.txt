cmake_minimum_required(VERSION 3.8)
project(plato_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plato_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/pid_controller.cpp
  src/interpolation.cpp
  src/joint_position_storage.cpp
  src/joint_state_ordering.cpp
  src/util.cpp
  src/yaml_helpers.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen yaml-cpp)

add_executable(joint_position_saver
  src/joint_position_saver_main.cpp
)
target_compile_features(joint_position_saver PRIVATE cxx_std_17)
target_link_libraries(joint_position_saver ${PROJECT_NAME})
ament_target_dependencies(joint_position_saver
  plato_interfaces
  rclcpp
  sensor_msgs
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS joint_position_saver
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_index_cpp Eigen3 yaml-cpp)

ament_package()
