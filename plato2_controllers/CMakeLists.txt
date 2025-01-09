cmake_minimum_required(VERSION 3.16)
project(plato2_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

# Using this instead of visibility macros
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add the library
add_library(plato2_controllers SHARED
  src/joint_impedance_controller.cpp
)
target_compile_features(plato2_controllers PUBLIC cxx_std_17)
target_include_directories(plato2_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/plato2_controllers>
)
ament_target_dependencies(plato2_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface joint_impedance_command_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Test for loading the controller
  # ament_add_gmock(test_load_joint_impedance_controller
  #   test/test_load_joint_impedance_controller.cpp
  # )
  # target_link_libraries(test_load_joint_impedance_controller
  #   plato2_controllers
  # )
  # ament_target_dependencies(test_load_joint_impedance_controller
  #   controller_manager
  #   hardware_interface
  #   ros2_control_test_assets
  # )

  # Test the controller functionality
  # ament_add_gmock(test_joint_impedance_controller
  #   test/test_joint_impedance_controller.cpp
  # )
  # target_link_libraries(test_joint_impedance_controller
  #   plato2_controllers
  # )
endif()

install(
  DIRECTORY include/
  DESTINATION include/plato2_controllers
)
install(
  TARGETS
    plato2_controllers
  EXPORT export_plato2_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(
  FILES joint_impedance_command_plugin.xml
  DESTINATION share/plato2_controllers
)

ament_export_targets(export_plato2_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
