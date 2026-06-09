cmake_minimum_required(VERSION 3.8)
project(plato_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_program(XACRO_EXECUTABLE xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

################################################################################
# Build
################################################################################
# include_directories(
#   include
# )

set(GENERATED_URDF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/urdf/generated")
file(MAKE_DIRECTORY "${GENERATED_URDF_DIR}")

function(generate_urdf_from_xacro target_name xacro_file urdf_file)
  add_custom_command(
    OUTPUT "${GENERATED_URDF_DIR}/${urdf_file}"
    COMMAND
      "${CMAKE_COMMAND}" -E env
      "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
      "${XACRO_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/${xacro_file}"
      -o "${GENERATED_URDF_DIR}/${urdf_file}"
    DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/${xacro_file}"
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/ros2_control/aristo.ros2_control.xacro"
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xacro/aristo_description.xacro"
    COMMENT "Generating ${urdf_file} from ${xacro_file}"
    VERBATIM
  )
  add_custom_target(${target_name} ALL DEPENDS "${GENERATED_URDF_DIR}/${urdf_file}")
endfunction()

generate_urdf_from_xacro(generate_aristo_urdf aristo.urdf.xacro aristo.urdf)

install(DIRECTORY 
  launch 
  urdf 
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
