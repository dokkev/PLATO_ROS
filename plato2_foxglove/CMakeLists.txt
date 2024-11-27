cmake_minimum_required(VERSION 3.8)
project(plato2_foxglove)

# Ensure that the package uses ament_cmake
find_package(ament_cmake REQUIRED)

# Find any other dependencies
# Uncomment and add required dependencies as needed
# find_package(<dependency> REQUIRED)

# Install Python scripts (e.g., your nodes)
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files if present
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linting and tests
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
