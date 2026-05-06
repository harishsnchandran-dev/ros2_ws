# UR5 Collaborative Robot Safety System
ROS 2 Humble · MoveIt2 · Python · Gazebo · Ubuntu 22.04

## What Problem Does This Solve?
In collaborative manufacturing environments, human workers and robot arms 
operate in shared spaces. Without real-time safety monitoring, a robot arm 
can cause serious injury if a worker enters its motion path unexpectedly. 
This system uses computer vision to detect human presence in the UR5's 
workspace and instantly halts or reroutes the arm — preventing accidents 
before they happen.

## Demo
▶ [Demo video coming soon]

## Architecture
Camera → Safety Monitor Node → MoveIt2 Planner → UR5 Driver

## Tech Stack
- ROS 2 Humble
- MoveIt2 (motion planning)
- Python 3.10
- Gazebo Simulation
- Ubuntu 22.04

## How to Run
git clone [your repo URL]
cd ros2_ws && colcon build
source install/setup.bash
ros2 launch ur5_safety_system demo.launch.py

## Results
- Real-time human presence detection in robot workspace
- Arm halt response triggered within [X] ms of detection
- Tested across [X] simulation scenarios with zero collision events
