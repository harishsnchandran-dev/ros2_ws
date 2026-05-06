#!/bin/bash

# UR5 Simulation Menu script
# Usage: ./test_sim.sh

echo "=== UR5 Control Suite Launcher ==="
echo "1. Launch Simulation (Gazebo + MoveIt2)"
echo "2. Pick and Place 90 Degrees (Single Cycle)"
echo "3. Pick and Place 90 Degrees (INFINITE LOOP)"
echo "4. Clean and Build workspace"
echo "5. Run IK Test"
echo "=================================="
read -p "Choose an option: " opt

case $opt in
  1)
    echo "Launching Simulation..."
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch ur5_description ur5_sim.launch.py
    ;;
  2)
    echo "Running 90-Degree Pick and Place..."
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run ur5_description pick_and_place_90.py
    ;;
  3)
    echo "Running INFINITE LOOP Pick and Place..."
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run ur5_description pick_and_place_loop.py
    ;;
  4)
    echo "Building workspace..."
    colcon build --packages-select ur5_description
    ;;

  5)
    echo "Running MoveIt IK Test..."
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run ur5_description moveit_ik_test
    ;;
  *)
    echo "Invalid option."
    ;;
esac
