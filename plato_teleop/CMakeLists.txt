cmake_minimum_required(VERSION 3.8)
project(plato_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package and entry points (configured in setup.py)
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes where launch_ros expects package executables.
install(PROGRAMS plato_teleop/spacemouse_twist.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spacemouse_twist)

install(PROGRAMS plato_teleop/spacemouse_state_machine.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spacemouse_state_machine)

install(PROGRAMS plato_teleop/impedance_gain_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME impedance_gain_keyboard)

install(PROGRAMS plato_teleop/thumb_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME thumb_keyboard_teleop)

install(PROGRAMS plato_teleop/retargeting_converter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME retargeting_converter)

install(PROGRAMS plato_teleop/ftip_to_parallel_grasp.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ftip_to_parallel_grasp)

# Install launch files and package manifest
install(DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
