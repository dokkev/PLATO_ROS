cmake_minimum_required(VERSION 3.16)
project(aristo_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  mppi_core
  plato_robot_system
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/config/aristo_config.cpp
  src/state_machines/grasp_force.cpp
  src/state_machines/grasp_teleop.cpp
  src/state_machines/grasp_ready.cpp
  src/state_machines/idle.cpp
  src/state_machines/initialize.cpp
  src/state_machines/joint_teleop.cpp
  src/state_machines/mppi_grasp.cpp
  src/state_machines/mppi_motion_grasp.cpp
  src/state_machines/poke.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(ARISTO_CONTROLLER_CONFIG_PATH
    "${CMAKE_CURRENT_SOURCE_DIR}/config/aristo.yaml")

  ament_add_gtest(aristo_config_test
    test/aristo_config_test.cpp
  )
  if(TARGET aristo_config_test)
    target_link_libraries(aristo_config_test ${PROJECT_NAME})
    target_compile_definitions(aristo_config_test PRIVATE
      ARISTO_CONTROLLER_CONFIG_PATH=\"${ARISTO_CONTROLLER_CONFIG_PATH}\")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} yaml-cpp)
ament_package()
