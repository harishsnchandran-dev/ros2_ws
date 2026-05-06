cmake_minimum_required(VERSION 3.8)
project(ur5_description)

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

# Install all resource directories
install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/pick_and_place_90.py
  scripts/pick_and_place_loop.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_package sets up a regular package
ament_package()
