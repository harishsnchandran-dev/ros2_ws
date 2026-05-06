"""
safety_operate.launch.py — Run AFTER ur5_sim.launch.py is fully up.

Uses the ORIGINAL pick_and_place_loop.py from ur5_description (proven working)
and layers the safety zone manager on top of it. The original script already
supports the speed_multiplier parameter — we just dynamically set it based
on human distance.

Step 1:  ros2 launch ur5_description ur5_sim.launch.py
         (wait ~15s for Gazebo + MoveIt2 to start)

Step 2:  ros2 launch ur5_safety_cobot safety_operate.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    safety_pkg = get_package_share_directory('ur5_safety_cobot')
    safety_config = os.path.join(safety_pkg, 'config', 'safety_zones.yaml')

    # ── Safety Zone Manager (targets the original pnp_loop node) ──────────
    zone_manager_node = Node(
        package='ur5_safety_cobot',
        executable='safety_zone_manager.py',
        name='safety_zone_manager',
        output='screen',
        parameters=[safety_config],
    )

    # ── Original Pick-and-Place from ur5_description (PROVEN WORKING) ─────
    pnp_node = Node(
        package='ur5_description',
        executable='pick_and_place_loop.py',
        name='pnp_loop',
        output='screen',
    )
    delayed_pnp = TimerAction(period=3.0, actions=[pnp_node])

    # ── Dashboard ─────────────────────────────────────────────────────────
    dashboard_node = Node(
        package='ur5_safety_cobot',
        executable='safety_dashboard.py',
        name='safety_dashboard',
        output='screen',
    )

    return LaunchDescription([
        zone_manager_node,
        delayed_pnp,
        dashboard_node,
    ])
