cmake_minimum_required(VERSION 3.8)
project(plato_robot_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/control/fsm_handler.cpp
  src/control/pid_controller.cpp
  src/control/plato_control_architecture.cpp
  src/control/state_machine.cpp
  src/robot/robot_system.cpp
  src/sensor/nari_touch.cpp
  src/sensor/nari_touch_adapter.cpp
  src/sensor/tactile_grip_observation.cpp
  src/sensor/tactile_state.cpp
  src/task/grasp_idqp.cpp
  src/task/grasp_idqp_parallel_solver.cpp
  src/task/grasp_task.cpp
  src/task/joint_teleop_task.cpp
  src/task/joint_task.cpp
  src/trajectory/euclidean_trajectory.cpp
  src/trajectory/interpolation.cpp
  src/trajectory/joint_integrator.cpp
  src/trajectory/se3_trajectory.cpp
  src/trajectory/trajectory_base.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  set(ARISTO_GENERATED_URDF
    "${CMAKE_CURRENT_SOURCE_DIR}/../plato_description/urdf/generated/aristo.urdf")

  ament_add_gtest(grasp_task_test
    test/grasp_task_test.cpp
  )
  if(TARGET grasp_task_test)
    target_link_libraries(grasp_task_test ${PROJECT_NAME})
    target_compile_definitions(grasp_task_test PRIVATE
      PLATO_ROBOT_SYSTEM_ARISTO_URDF_PATH=\"${ARISTO_GENERATED_URDF}\")
  endif()

  ament_add_gtest(grasp_idqp_test
    test/grasp_idqp_test.cpp
  )
  if(TARGET grasp_idqp_test)
    target_link_libraries(grasp_idqp_test ${PROJECT_NAME})
    target_compile_definitions(grasp_idqp_test PRIVATE
      PLATO_ROBOT_SYSTEM_ARISTO_URDF_PATH=\"${ARISTO_GENERATED_URDF}\")
  endif()

  ament_add_gtest(joint_teleop_task_test
    test/joint_teleop_task_test.cpp
  )
  if(TARGET joint_teleop_task_test)
    target_link_libraries(joint_teleop_task_test ${PROJECT_NAME})
  endif()

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 pinocchio)
ament_package()
