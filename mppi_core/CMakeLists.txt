cmake_minimum_required(VERSION 3.8)
project(mppi_core)

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
find_package(plato_robot_system REQUIRED)
find_package(yaml-cpp REQUIRED)

set(${PROJECT_NAME}_SOURCES
  src/core/action_sequence.cpp
  src/core/mppi_optimizer.cpp
  src/costs/grasp_stability_cost.cpp
  src/rollout/grasp_state_rollout_model.cpp
  src/util/mppi_config.cpp
  src/util/rollout_config.cpp
  src/util/task_config.cpp
  src/util/yaml_utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  plato_robot_system::plato_robot_system
  Eigen3::Eigen
  pinocchio::pinocchio
  yaml-cpp
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

install(DIRECTORY config task
  DESTINATION share/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(plato_description REQUIRED)

  find_program(XACRO_EXECUTABLE xacro)
  if(NOT XACRO_EXECUTABLE)
    message(FATAL_ERROR "xacro executable is required for mppi_core tests")
  endif()

  get_filename_component(
    PLATO_DESCRIPTION_SHARE_DIR "${plato_description_DIR}/.." ABSOLUTE)
  get_filename_component(
    PLATO_DESCRIPTION_PREFIX "${plato_description_DIR}/../../.." ABSOLUTE)
  set(PLATO_NARITOUCH_XACRO
    "${PLATO_DESCRIPTION_SHARE_DIR}/urdf/plato_naritouch.urdf.xacro")
  set(PLATO_NARITOUCH_TEST_URDF
    "${CMAKE_CURRENT_BINARY_DIR}/plato_naritouch.urdf")

  add_custom_command(
    OUTPUT "${PLATO_NARITOUCH_TEST_URDF}"
    COMMAND ${CMAKE_COMMAND} -E env
      "AMENT_PREFIX_PATH=${PLATO_DESCRIPTION_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
      "${XACRO_EXECUTABLE}"
      -o "${PLATO_NARITOUCH_TEST_URDF}"
      "${PLATO_NARITOUCH_XACRO}"
      "fake_hardware:=true"
      "zeroing:=false"
    DEPENDS
      "${PLATO_NARITOUCH_XACRO}"
      "${PLATO_DESCRIPTION_SHARE_DIR}/urdf/xacro/plato_naritouch_description.xacro"
      "${PLATO_DESCRIPTION_SHARE_DIR}/urdf/ros2_control/plato.ros2_control.xacro"
    VERBATIM
  )
  add_custom_target(plato_naritouch_urdf_for_test
    DEPENDS "${PLATO_NARITOUCH_TEST_URDF}")

  ament_add_gtest(test_mppi_core
    test/test_mppi_core.cpp
  )
  if(TARGET test_mppi_core)
    add_dependencies(test_mppi_core plato_naritouch_urdf_for_test)
    target_link_libraries(test_mppi_core ${PROJECT_NAME})
    target_compile_definitions(test_mppi_core PRIVATE
      MPPI_CORE_PLATO_NARITOUCH_URDF_PATH=\"${PLATO_NARITOUCH_TEST_URDF}\")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 pinocchio plato_robot_system yaml-cpp)

ament_package()
