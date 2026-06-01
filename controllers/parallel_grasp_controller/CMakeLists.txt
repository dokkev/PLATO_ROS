cmake_minimum_required(VERSION 3.8)
project(parallel_grasp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(plato_interfaces REQUIRED)
find_package(plato_utils REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(parallel_grasp_node src/parallel_grasp_node.cpp src/parallel_grasp_controller.cpp)
target_include_directories(parallel_grasp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(parallel_grasp_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(parallel_grasp_node
  rclcpp
  std_msgs
  sensor_msgs
  plato_interfaces
  plato_utils
  Eigen3
)

install(TARGETS parallel_grasp_node
  DESTINATION lib/${PROJECT_NAME})


# Install launch files and config
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
