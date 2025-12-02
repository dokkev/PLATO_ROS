cmake_minimum_required(VERSION 3.8)
project(joint_position_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(plato2_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(plato_utils REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(joint_position_controller_node
  src/control_node.cpp
  src/joint_position_controller.cpp
  src/impedance_gain_handler.cpp
)
target_include_directories(joint_position_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_position_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(joint_position_controller_node
  rclcpp
  std_msgs
  plato2_interfaces
  sensor_msgs
  std_srvs
  plato_utils
)
target_link_libraries(joint_position_controller_node Eigen3::Eigen)

install(TARGETS joint_position_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
# Install launch files and config if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

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
