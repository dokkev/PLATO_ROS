cmake_minimum_required(VERSION 3.8)
project(plato_hardware_interface LANGUAGES CXX)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  can_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  plato_hardware_interface
  SHARED
  src/plato.cpp
  src/plato_socket_can.cpp
  src/plato_motor_direction.cpp
  src/socket_can.cpp
  src/epoll_event_loop.cpp
)
target_compile_features(plato_hardware_interface PUBLIC cxx_std_17)
target_include_directories(plato_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/plato_hardware_interface>
)
include_directories("/include/plato_hardware_interface") 

ament_target_dependencies(
  plato_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PLATO_HARDWARE_INTERFACE_BUILDING_DLL")

# Export Hardware Interface
pluginlib_export_plugin_description_file(hardware_interface plato_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/plato_hardware_interface
)



install(DIRECTORY 
  description/ros2_control
  description/urdf
  rviz
  config
  launch 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS plato_hardware_interface
  EXPORT export_plato_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

## EXPORTS
ament_export_targets(export_plato_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
