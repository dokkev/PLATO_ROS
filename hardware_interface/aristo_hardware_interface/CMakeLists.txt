cmake_minimum_required(VERSION 3.8)
project(aristo_hardware_interface LANGUAGES CXX)

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
  geometry_msgs
  plato_interfaces
  plato_hardware_interface
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
  src/mit_can_protocol.cpp
  src/aristo_hand.cpp
  src/aristo_model.cpp
  src/aristo_protocol.cpp
  src/aristo.cpp
  src/utils/actuator_config_loader.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  can_hardware_common::can_hardware_common
  plato_hardware_interface::plato_hardware_interface
  yaml-cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ARISTO_HARDWARE_INTERFACE_BUILDING_DLL)

pluginlib_export_plugin_description_file(hardware_interface aristo_hardware_interface.xml)

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

  ament_add_gtest(aristo_protocol_test
    test/aristo_protocol_test.cpp
  )
  target_link_libraries(aristo_protocol_test
    ${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_package()
