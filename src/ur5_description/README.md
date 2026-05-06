# UR5 Collaborative Robot Safety System
ROS 2 Humble · MoveIt2 · Python · Gazebo · Ubuntu 22.04

## What Problem Does This Solve?
[2-3 sentences: what safety risk does your system address? 
what happens in a factory without it?]

## Demo
▶ [Demo video coming soon]

## Architecture
Camera → Safety Monitor Node → MoveIt2 Planner → UR5 Driver

## Tech Stack
- ROS 2 Humble · MoveIt2 · Python 3.10
- Gazebo Simulation · Ubuntu 22.04

## How to Run
git clone [your repo URL]
cd ros2_ws && colcon build
source install/setup.bash
ros2 launch ur5_safety_system demo.launch.py

## Results
[What did your system achieve? Response time? Accuracy?]
