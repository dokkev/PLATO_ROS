cmake_minimum_required(VERSION 3.8)
project(plato2_hardware_interface LANGUAGES CXX)

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
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)


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
  src/utils/ring_buf.cpp
  src/pcan_interface.cpp
  src/ft_sensor.cpp
  src/can_protocol.cpp
  src/mit_can_protocol.cpp
  src/actuator.cpp
  src/karnopp_compensator.cpp
  src/plato2_hand.cpp
  src/plato2.cpp
  
)
target_compile_features(plato2_hardware_interface PUBLIC cxx_std_17)
target_include_directories(plato2_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/plato2_hardware_interface>
${PCAN_INCLUDE_DIR}
/usr/include/eigen3
/usr/local/include/eigen3
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

# Diagnostic tool to verify motor replies on the bus
add_executable(can_verify src/tools/can_verify.cpp)
target_include_directories(can_verify PRIVATE ${PCAN_INCLUDE_DIR})
target_link_libraries(can_verify plato2_hardware_interface ${PCAN_LIB})
install(TARGETS can_verify RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)
  
  # MIT CAN Protocol unit test
  ament_add_gtest(test_mit_can_protocol test/test_mit_can_protocol.cpp)
  target_include_directories(test_mit_can_protocol PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCAN_INCLUDE_DIR}
  )
  target_link_libraries(test_mit_can_protocol 
    plato2_hardware_interface
    ${PCAN_LIB}
  )
endif()

## EXPORTS
ament_export_targets(export_plato2_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
