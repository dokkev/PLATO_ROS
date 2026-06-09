cmake_minimum_required(VERSION 3.8)
project(aristo_grasp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(plato_robot_system REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plato_description REQUIRED)
find_package(plato_interfaces REQUIRED)
find_package(plato_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(aristo_grasp_controller_core STATIC
  src/aristo_grasp_controller.cpp
  src/aristo_kinematics.cpp
)

target_include_directories(aristo_grasp_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${plato_robot_system_INCLUDE_DIRS}
)

target_link_libraries(aristo_grasp_controller_core
  plato_robot_system::plato_robot_system
  Eigen3::Eigen
  pinocchio::pinocchio
)

ament_target_dependencies(aristo_grasp_controller_core
  Eigen3
  pinocchio
)

add_executable(aristo_grasp_node
  src/aristo_grasp_node.cpp
)
target_link_libraries(aristo_grasp_node aristo_grasp_controller_core)
ament_target_dependencies(aristo_grasp_node
  ament_index_cpp
  plato_interfaces
  plato_utils
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS aristo_grasp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
