cmake_minimum_required(VERSION 3.8)
project(plato_grasp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_impedance_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plato_interfaces REQUIRED)
find_package(plato_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(plato_grasp_controller_core STATIC
  src/grasp_plan_parsing.cpp
  src/plato_grasp_planner.cpp
  src/plato_grasp_impedance_handler.cpp
  src/plato_grasp_task_runner.cpp
  src/plato_grasp_controller_node.cpp
)

target_include_directories(plato_grasp_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(plato_grasp_controller_core PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(plato_grasp_controller_core
  joint_impedance_controller
  rclcpp
  plato_interfaces
  plato_utils
  sensor_msgs
  std_msgs
)
target_link_libraries(plato_grasp_controller_core
  joint_impedance_controller::joint_impedance_controller_impedance_handler
  yaml-cpp
)

add_executable(plato_grasp_controller_node
  src/plato_grasp_controller_main.cpp
)
target_compile_features(plato_grasp_controller_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(plato_grasp_controller_node plato_grasp_controller_core)
ament_target_dependencies(plato_grasp_controller_node rclcpp)

add_executable(joint_position_saver
  src/joint_position_saver_main.cpp
)
target_compile_features(joint_position_saver PUBLIC c_std_99 cxx_std_17)
target_link_libraries(joint_position_saver plato_grasp_controller_core)
ament_target_dependencies(joint_position_saver rclcpp)

install(TARGETS plato_grasp_controller_node joint_position_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(plato_grasp_controller_core_test
    test/plato_grasp_controller_core_test.cpp
  )
  if(TARGET plato_grasp_controller_core_test)
    target_include_directories(plato_grasp_controller_core_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_features(plato_grasp_controller_core_test PUBLIC cxx_std_17)
    target_link_libraries(plato_grasp_controller_core_test
      plato_grasp_controller_core
    )
    ament_target_dependencies(plato_grasp_controller_core_test
      rclcpp
      plato_interfaces
      plato_utils
      sensor_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(plato_grasp_controller_pipeline_test
    test/plato_grasp_controller_pipeline_test.cpp
  )
  if(TARGET plato_grasp_controller_pipeline_test)
    target_include_directories(plato_grasp_controller_pipeline_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_features(plato_grasp_controller_pipeline_test PUBLIC cxx_std_17)
    target_link_libraries(plato_grasp_controller_pipeline_test
      plato_grasp_controller_core
    )
    ament_target_dependencies(plato_grasp_controller_pipeline_test
      rclcpp
      plato_interfaces
      plato_utils
      sensor_msgs
      std_msgs
    )
  endif()
endif()

ament_package()
