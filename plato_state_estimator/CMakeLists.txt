cmake_minimum_required(VERSION 3.8)
project(plato_state_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mppi_core REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdr_grasp_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Build core library
add_library(reference_grasp_force_generator SHARED
  src/reference_grasp_force_generator.cpp
)

# Add executables
add_executable(contact_estimator src/contact_estimator.cpp)
ament_target_dependencies(contact_estimator
  rclcpp
  geometry_msgs
  std_msgs)

add_executable(object_state_estimator_node src/reference_grasp_force_generator_node.cpp)
target_link_libraries(object_state_estimator_node
  reference_grasp_force_generator
)
ament_target_dependencies(object_state_estimator_node
  rclcpp
  geometry_msgs
  std_msgs
  sdr_grasp_msgs)

add_executable(object_prior_estimator_node src/object_prior_estimator_node.cpp)
ament_target_dependencies(object_prior_estimator_node
  rclcpp
  geometry_msgs
  mppi_core
  pinocchio
  sensor_msgs
  std_msgs
  sdr_grasp_msgs
  visualization_msgs)

# Install library
install(TARGETS reference_grasp_force_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  contact_estimator
  object_state_estimator_node
  object_prior_estimator_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch and config files
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(reference_grasp_force_generator_test
    test/reference_grasp_force_generator_test.cpp
  )
  if(TARGET reference_grasp_force_generator_test)
    target_link_libraries(reference_grasp_force_generator_test
      reference_grasp_force_generator)
  endif()
endif()

ament_package()
