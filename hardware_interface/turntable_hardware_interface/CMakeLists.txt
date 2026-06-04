cmake_minimum_required(VERSION 3.8)
project(turntable_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  dynamixel_sdk
  rclcpp
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/dynamixel.cpp
  src/min_jerk_traj.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  PUBLIC dynamixel_sdk::dynamixel_sdk
)

add_executable(turntable_dynamixel_node
  src/turntable_dynamixel_node.cpp
)
target_compile_features(turntable_dynamixel_node PUBLIC cxx_std_17)
target_include_directories(turntable_dynamixel_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(turntable_dynamixel_node
  ament_index_cpp
  rclcpp
  std_msgs
)
target_link_libraries(turntable_dynamixel_node
  ${PROJECT_NAME}
  yaml-cpp
)

add_executable(turntable_dynamixel_scan
  src/turntable_dynamixel_scan.cpp
)
target_compile_features(turntable_dynamixel_scan PUBLIC cxx_std_17)
ament_target_dependencies(turntable_dynamixel_scan
  dynamixel_sdk
)
target_link_libraries(turntable_dynamixel_scan
  dynamixel_sdk::dynamixel_sdk
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  turntable_dynamixel_node
  turntable_dynamixel_scan
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(min_jerk_traj_test
    test/min_jerk_traj_test.cpp
  )
  if(TARGET min_jerk_traj_test)
    target_compile_features(min_jerk_traj_test PUBLIC cxx_std_17)
    target_include_directories(min_jerk_traj_test PRIVATE
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    )
    target_link_libraries(min_jerk_traj_test ${PROJECT_NAME})
  endif()

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_package()
