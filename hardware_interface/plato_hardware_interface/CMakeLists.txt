cmake_minimum_required(VERSION 3.8)
project(plato_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  can_hardware_common
  rclcpp
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
  src/utils/actuator_config_loader.cpp
  src/utils/actuator_offset_loader.cpp
  src/utils/linkage_config_loader.cpp
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

# Reference-only files kept in source tree, but not built or installed:
#   - plato.cpp / plato.hpp
#   - plato_hand.cpp / plato_hand.hpp / plato_hand_config.hpp
#   - plato_model.cpp / plato_model.hpp
#   - plato_protocol.cpp / plato_protocol.hpp
#   - utils/plato_hand_config_loader.cpp / .hpp

install(DIRECTORY include/
  DESTINATION include
  PATTERN "plato.hpp" EXCLUDE
  PATTERN "plato_hand.hpp" EXCLUDE
  PATTERN "plato_hand_config.hpp" EXCLUDE
  PATTERN "plato_model.hpp" EXCLUDE
  PATTERN "plato_protocol.hpp" EXCLUDE
  PATTERN "plato_hand_config_loader.hpp" EXCLUDE
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} yaml-cpp)
ament_export_libraries(${PROJECT_NAME})
ament_package()
