import os
import xacro
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                             TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# The URDF is written to /tmp at import time so spawn_entity can use -file (no QoS issues)
_pkg_path = get_package_share_directory('ur5_description')
_xacro_file = os.path.join(_pkg_path, 'urdf', 'ur5.urdf.xacro')
import re
_robot_description_config = xacro.process_file(_xacro_file)
_urdf_string = _robot_description_config.toxml()
# Minify URDF to completely bypass rclcpp parser limits in URDF injection
_urdf_string = re.sub(r'<!--.*?-->', '', _urdf_string, flags=re.DOTALL)
_urdf_string = re.sub(r'>\s+<', '><', _urdf_string)
_urdf_string = _urdf_string.replace('\n', '').replace('\r', '')

TMP_URDF = '/tmp/ur5_gazebo.urdf'
with open(TMP_URDF, 'w') as _f:
    _f.write(_urdf_string)


def generate_launch_description():

    robot_description = {'robot_description': _urdf_string}

    # ── Gazebo (Classic) ─────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'false', 'pause': 'false'}.items(),
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ── Spawn robot from topic (keeps ros2_control tags intact) ───────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ur5',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # Delay spawn 4 s so gzserver is definitely accepting connections
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_entity])

    # ── Controller spawners (sequential, triggered by spawn completion) ───────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        load_jsb,
        load_jtc,
    ])