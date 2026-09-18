cmake_minimum_required(VERSION 3.8...3.26)
project(costmap_cspace_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
# rviz builds its plugins without the Qt keywords, so the sources here use the
# Q_SLOTS / Q_SIGNALS / Q_EMIT macro spellings.
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/c_space_3d_display.hpp
  src/c_space_3d_display.cpp
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # CPP_FORMATTER=clang_format: exclude uncrustify, keep clang_format + cpplint.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  # The BSD headers carried over from rviz name Willow Garage where
  # ament_copyright's templates expect "the copyright holder", so it reports
  # license=<unknown>. The linter is excluded rather than the license reworded.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
endif()

# USE_SCOPED_HEADER_INSTALL_DIR: install headers under include/${PROJECT_NAME}/
# rather than include/. This is the ament_auto_package default from Kilted on;
# opting in now keeps the install layout stable across the upgrade.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
