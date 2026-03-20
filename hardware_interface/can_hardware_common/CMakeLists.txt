cmake_minimum_required(VERSION 3.8)
project(can_hardware_common LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_library(PCAN_LIB NAMES pcanbasic PATHS /usr/local/lib /usr/lib /lib REQUIRED)
find_path(PCAN_INCLUDE_DIR NAMES PCANBasic.h PATHS /usr/local/include /usr/include REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can_bus_manager.cpp
  src/ft_sensor.cpp
  src/pcan_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCAN_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp)
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC ${PCAN_LIB}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 geometry_msgs rclcpp)
ament_export_libraries(${PROJECT_NAME})
ament_package()
