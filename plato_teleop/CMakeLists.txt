cmake_minimum_required(VERSION 3.8)
project(plato_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  pluginlib
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

# add_executable(moveit_control src/moveit_control.cpp)
# add_executable(state_and_model src/state_and_model.cpp)
# add_executable(motion_planning src/motion_planning.cpp)
# add_executable(controller_WholeRobots src/controller_WholeRobots.cpp)
# add_executable(controller_PlatoHand src/controller_PlatoHand.cpp)
add_executable(controller_spaceMouse src/controller_spaceMouse.cpp)

# target_include_directories(state_and_model
#     PUBLIC include)
# target_include_directories(motion_planning
#     PUBLIC include)
# target_include_directories(controller_WholeRobots
#     PUBLIC include)
# target_include_directories(controller_PlatoHand
#     PUBLIC include)
target_include_directories(controller_spaceMouse
    PUBLIC include)

# ament_target_dependencies(rclcpp std_msgs)

# ament_target_dependencies(state_and_model
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# ament_target_dependencies(motion_planning
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# ament_target_dependencies(controller_WholeRobots
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
# ament_target_dependencies(controller_PlatoHand
#     ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(controller_spaceMouse
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# install(TARGETS moveit_control 
#     DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS state_and_model 
#     DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS motion_planning
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS controller_WholeRobots
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS controller_PlatoHand
#   DESTINATION lib/${PROJECT_NAME}
# )
install(TARGETS controller_spaceMouse
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



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

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(include)


ament_package()