cmake_minimum_required(VERSION 3.8...3.26)
project(neonavigation_rviz_plugins)

find_package(ament_cmake REQUIRED)

# Nothing here but the manifest and this file, so only their linters apply;
# ament_lint_auto would also run source linters that find no files.
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
