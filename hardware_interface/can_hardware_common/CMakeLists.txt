cmake_minimum_required(VERSION 3.8)
project(can_hardware_common LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

find_library(PCAN_LIB NAMES pcanbasic PATHS /usr/local/lib /usr/lib /lib REQUIRED)
find_path(PCAN_INCLUDE_DIR NAMES PCANBasic.h PATHS /usr/local/include /usr/include REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/command_scheduler.cpp
  src/can_transport.cpp
  src/pcan_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCAN_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
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

install(PROGRAMS
  script/pcan_hardware_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(command_scheduler_test
    test/command_scheduler_test.cpp
  )
  target_link_libraries(command_scheduler_test
    ${PROJECT_NAME}
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_package()
