"""
safety_sim.launch.py — Launches the UR5 simulation stack.

Simply includes the proven ur5_sim.launch.py from ur5_description which
starts: Gazebo + Robot State Publisher + Controllers + MoveIt2 + RViz2

Usage:  ros2 launch ur5_safety_cobot safety_sim.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ur5_pkg = get_package_share_directory('ur5_description')

    ur5_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5_pkg, 'launch', 'ur5_sim.launch.py')
        )
    )

    return LaunchDescription([ur5_sim])

