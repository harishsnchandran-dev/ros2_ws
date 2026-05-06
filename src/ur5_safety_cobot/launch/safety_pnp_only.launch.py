"""
safety_pnp_only.launch.py — PnP + Safety (no camera / detector).

For testing without a physical USB camera. You can manually publish
distance values to /human_distance to test zone transitions:

  ros2 topic pub /human_distance std_msgs/msg/Float32 "data: 150.0"
  ros2 topic pub /human_distance std_msgs/msg/Float32 "data: 80.0"
  ros2 topic pub /human_distance std_msgs/msg/Float32 "data: 250.0"

Usage:  ros2 launch ur5_safety_cobot safety_pnp_only.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ur5_safety_cobot')
    safety_config = os.path.join(pkg, 'config', 'safety_zones.yaml')

    zone_manager = Node(
        package='ur5_safety_cobot',
        executable='safety_zone_manager.py',
        name='safety_zone_manager',
        output='screen',
        parameters=[safety_config],
    )

    pnp_node = Node(
        package='ur5_safety_cobot',
        executable='safety_pick_and_place.py',
        name='safety_pnp_node',
        output='screen',
    )
    delayed_pnp = TimerAction(period=3.0, actions=[pnp_node])

    dashboard = Node(
        package='ur5_safety_cobot',
        executable='safety_dashboard.py',
        name='safety_dashboard',
        output='screen',
    )

    return LaunchDescription([
        zone_manager,
        delayed_pnp,
        dashboard,
    ])
