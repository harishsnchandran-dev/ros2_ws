cmake_minimum_required(VERSION 3.8)
project(ur5_safety_cobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install resource directories
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/safety_pick_and_place.py
  scripts/safety_zone_manager.py
  scripts/safety_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
