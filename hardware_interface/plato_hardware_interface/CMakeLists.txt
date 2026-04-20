cmake_minimum_required(VERSION 3.8)
project(plato_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  can_hardware_common
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_msgs
  plato_interfaces
  Eigen3
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/actuator.cpp
  src/can_protocol.cpp
  src/five_bar_linkage.cpp
  src/plato_hand.cpp
  src/plato_model.cpp
  src/plato_protocol.cpp
  src/plato.cpp
  src/utils/actuator_config_loader.cpp
  src/utils/actuator_offset_loader.cpp
  src/utils/linkage_config_loader.cpp
  src/utils/plato_hand_config_loader.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PUBLIC can_hardware_common::can_hardware_common
  PUBLIC yaml-cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE PLATO_HARDWARE_INTERFACE_BUILDING_DLL)

find_package(plato_utils REQUIRED)

# TODO: Update profiler to use new CanTransport/CanCommandScheduler API
# add_executable(can_rate_profiler src/can_rate_profiler_node.cpp)
# target_link_libraries(can_rate_profiler ${PROJECT_NAME})
# target_include_directories(can_rate_profiler PRIVATE ${plato_utils_INCLUDE_DIRS})
# ament_target_dependencies(can_rate_profiler rclcpp plato_utils)
# install(TARGETS can_rate_profiler DESTINATION lib/${PROJECT_NAME})

add_executable(plato_hand_sim_smoke_test
  src/tools/plato_hand_sim_smoke_test.cpp
)
target_compile_features(plato_hand_sim_smoke_test PUBLIC cxx_std_17)
target_link_libraries(plato_hand_sim_smoke_test
  PRIVATE ${PROJECT_NAME}
)

add_executable(plato_hand_hil_enable_zero_test
  src/tools/plato_hand_hil_enable_zero_test.cpp
)
target_compile_features(plato_hand_hil_enable_zero_test PUBLIC cxx_std_17)
target_link_libraries(plato_hand_hil_enable_zero_test
  PRIVATE ${PROJECT_NAME}
)

add_executable(plato_hand_hil_gap_sweep
  src/tools/plato_hand_hil_gap_sweep.cpp
)
target_compile_features(plato_hand_hil_gap_sweep PUBLIC cxx_std_17)
target_link_libraries(plato_hand_hil_gap_sweep
  PRIVATE ${PROJECT_NAME}
)

install(TARGETS plato_hand_sim_smoke_test
  plato_hand_hil_enable_zero_test
  plato_hand_hil_gap_sweep
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface plato_hardware_interface.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(plato_protocol_test
    test/plato_protocol_test.cpp
  )
  target_link_libraries(plato_protocol_test
    ${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_package()
