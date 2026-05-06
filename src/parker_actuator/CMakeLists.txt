cmake_minimum_required(VERSION 3.8)
project(parker_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add hardware interface library
add_library(
  parker_actuator
  SHARED
  src/parker_hardware.cpp
)
target_include_directories(
  parker_actuator
  PRIVATE
  include
)
ament_target_dependencies(
  parker_actuator
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  controller_manager
  sensor_msgs
  trajectory_msgs
)

# Install
install(
  TARGETS parker_actuator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts to lib/ so ros2 run can find them
install(
  PROGRAMS scripts/manual_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES parker_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface parker_hardware.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
