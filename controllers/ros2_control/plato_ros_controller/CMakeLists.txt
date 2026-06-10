cmake_minimum_required(VERSION 3.16)
project(plato_ros_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  aristo_controller
  controller_interface
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  plato_interfaces
  plato_robot_system
  sdr_grasp_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(plato_ros_controller SHARED
  src/plato_ros_controller.cpp
)
target_compile_features(plato_ros_controller PUBLIC cxx_std_17)
target_include_directories(plato_ros_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/plato_ros_controller>
)
ament_target_dependencies(plato_ros_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface plato_ros_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/plato_ros_controller
)

install(
  TARGETS plato_ros_controller
  EXPORT export_plato_ros_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_export_targets(export_plato_ros_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
