cmake_minimum_required(VERSION 3.8)
project(plato2_hardware_interface LANGUAGES CXX)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# if (NOT TARGET sockcanpp)
#     add_subdirectory(libsockcanpp ${CMAKE_CURRENT_BUILD_DIR}/libsockcanpp)
# endif()

# target_link_libraries(

#     sockcanpp
# )

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

# Find PCAN Basic library and include path
find_library(PCAN_LIB NAMES pcanbasic PATHS /path/to/bin /usr/local/lib /usr/lib /lib)
find_path(PCAN_INCLUDE_DIR NAMES PCANBasic.h PATHS /usr/local/include /usr/include)

if(NOT PCAN_LIB)
  message(FATAL_ERROR "PCANBasic library not found.")
endif()

if(NOT PCAN_INCLUDE_DIR)
  message(FATAL_ERROR "PCANBasic include directory not found.")
endif()

message(STATUS "PCANBasic library found at: ${PCAN_LIB}")
message(STATUS "PCANBasic include directory found at: ${PCAN_INCLUDE_DIR}")


## COMPILE
add_library(
  plato2_hardware_interface
  SHARED
  src/plato2.cpp
  src/pcan_interface.cpp
)
target_compile_features(plato2_hardware_interface PUBLIC cxx_std_17)
target_include_directories(plato2_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/plato2_hardware_interface>
${PCAN_INCLUDE_DIR}
)
include_directories("/include/plato2_hardware_interface") 

ament_target_dependencies(
  plato2_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link PCANBasic library
target_link_libraries(plato2_hardware_interface PRIVATE ${PCAN_LIB})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PLATO2_HARDWARE_INTERFACE_BUILDING_DLL")

# Export Hardware Interface
pluginlib_export_plugin_description_file(hardware_interface plato2_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/plato2_hardware_interface
)



install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS plato2_hardware_interface
  EXPORT export_plato2_hardware_interface
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
ament_export_targets(export_plato2_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
