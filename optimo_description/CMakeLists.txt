cmake_minimum_required(VERSION 3.8)
project(optimo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


################################################################################
# Build
################################################################################
# include_directories(
#   include
# )

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  urdf 
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)
# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()
