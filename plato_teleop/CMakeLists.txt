cmake_minimum_required(VERSION 3.8)
project(plato_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package and entry points (configured in setup.py)
ament_python_install_package(${PROJECT_NAME})

# Install launch files and package manifest
install(DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
