cmake_minimum_required(VERSION 3.8)
project(maestro_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(fingertip_dist_to_parallel_grasp
  src/fingertip_dist_to_parallel_grasp.cpp)
target_compile_features(fingertip_dist_to_parallel_grasp PUBLIC cxx_std_17)
ament_target_dependencies(fingertip_dist_to_parallel_grasp
  rclcpp
  std_msgs
)

add_executable(float8array_to_impedance_trajectory
  src/float8array_to_impedance_trajectory.cpp)
target_compile_features(float8array_to_impedance_trajectory PUBLIC cxx_std_17)
ament_target_dependencies(float8array_to_impedance_trajectory
  rclcpp
  std_msgs
)

install(TARGETS
  fingertip_dist_to_parallel_grasp
  float8array_to_impedance_trajectory
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
