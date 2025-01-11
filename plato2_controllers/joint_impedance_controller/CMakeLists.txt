cmake_minimum_required(VERSION 3.16)
project(joint_impedance_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

# Using this instead of visibility macros
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  control_msgs
  trajectory_msgs
  plato2_interfaces
  realtime_tools
  std_srvs
)

# Add this dependency
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate parameter library
generate_parameter_library(joint_impedance_controller_parameters
  src/joint_impedance_controller_parameters.yaml
)

# Add the library
add_library(joint_impedance_controller SHARED
  src/joint_impedance_controller.cpp
)
target_compile_features(joint_impedance_controller PUBLIC cxx_std_17)
target_include_directories(joint_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_impedance_controller>
)

target_link_libraries(joint_impedance_controller PUBLIC 
  joint_impedance_controller_parameters
)
ament_target_dependencies(joint_impedance_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface joint_impedance_controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
endif()

install(
  DIRECTORY include/
  DESTINATION include/joint_impedance_controller
)
install(
  TARGETS
    joint_impedance_controller
    joint_impedance_controller_parameters
  EXPORT export_joint_impedance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib

)

ament_export_targets(export_joint_impedance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
