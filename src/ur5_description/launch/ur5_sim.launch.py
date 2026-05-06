"""
ur5_sim.launch.py — Master launch file
Starts: Gazebo + ROS2 Control + MoveIt2 + RViz2
Usage:  ros2 launch ur5_description ur5_sim.launch.py
"""
import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                             TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(pkg_name, file_path):
    pkg_path = get_package_share_directory(pkg_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path) as f:
        return yaml.safe_load(f)


# ── Pre-process xacro & write URDF to /tmp (avoids QoS spawn issues) ─────────
_pkg_path = get_package_share_directory('ur5_description')
_xacro_file = os.path.join(_pkg_path, 'urdf', 'ur5.urdf.xacro')
import re
_urdf_string = xacro.process_file(_xacro_file).toxml()
# Minify URDF to completely bypass rclcpp parser limits in URDF injection
_urdf_string = re.sub(r'<!--.*?-->', '', _urdf_string, flags=re.DOTALL)
_urdf_string = re.sub(r'\s+', ' ', _urdf_string)
_urdf_string = _urdf_string.replace('> <', '><')
# Fix: replace any unresolved $(find ...) with absolute package path
# (Gazebo Harmonic reads the raw URDF — it does NOT call xacro/rospack find)
import re as _re
_controllers_yaml = os.path.join(_pkg_path, 'config', 'controllers.yaml')
_urdf_string = _re.sub(
    r'\$\(find [^)]+\)',
    _pkg_path,
    _urdf_string
)

TMP_URDF = '/tmp/ur5_gazebo.urdf'
with open(TMP_URDF, 'w') as _f:
    _f.write(_urdf_string)


def generate_launch_description():

    pkg_path = get_package_share_directory('ur5_description')

    robot_description = {'robot_description': _urdf_string}

    srdf_file = os.path.join(pkg_path, 'config', 'ur5.srdf')
    with open(srdf_file) as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml       = load_yaml('ur5_description', 'config/kinematics.yaml')
    joint_limits_yaml     = load_yaml('ur5_description', 'config/joint_limits.yaml')
    ompl_yaml             = load_yaml('ur5_description', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('ur5_description', 'config/moveit_controllers.yaml')

    # ── Gazebo (Harmonic) ──────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    # Bridge /clock and services to ROS 2
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity',
        '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity'
    ]
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ── Spawn robot from topic (no QoS races if using delayed spawn) ─────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur5',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_entity])

    # ── Controller spawners ───────────────────────────────────────────────────
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
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
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
            on_exit=[joint_trajectory_controller_spawner, gripper_controller_spawner],
        )
    )

    # ── MoveIt2 move_group (after controllers are up) ─────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            {'planning_pipelines': ['ompl']},
            ompl_yaml,
            {
                'moveit_manage_controllers': True,
                'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                'trajectory_execution.allowed_goal_duration_margin': 0.5,
                'trajectory_execution.allowed_start_tolerance': 0.01,
            },
            {
                'moveit_simple_controller_manager':
                    moveit_controllers_yaml['moveit_simple_controller_manager'],
                'moveit_controller_manager':
                    'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            },
            {'use_sim_time': True},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
        ],
    )

    delayed_move_group = TimerAction(period=12.0, actions=[move_group_node])

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_yaml},
            {'use_sim_time': True},
        ],
    )

    delayed_rviz = TimerAction(period=13.0, actions=[rviz_node])

    # ── Spawn Table and Cube ──────────────────────────────────────────────────
    # Table (Box: 0.8 x 0.8 x 0.1 at height 0.4)
    table_urdf = '<?xml version="1.0" ?><robot name="table"><link name="link"><visual><geometry><box size="0.8 0.8 0.1"/></geometry><material name="Brown"><color rgba="0.5 0.3 0.1 1.0"/></material></visual><collision><geometry><box size="0.8 0.8 0.1"/></geometry></collision><inertial><mass value="10"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link></robot>'
    spawn_table = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'table', '-string', table_urdf, '-x', '0.6', '-y', '0.0', '-z', '1.2'],
        output='screen',
    )
    # Cube (0.04m Red Cube on the table)
    cube_urdf = '<?xml version="1.0" ?><robot name="cube"><link name="link"><visual><geometry><box size="0.04 0.04 0.04"/></geometry><material name="Red"><color rgba="1.0 0.0 0.0 1.0"/></material></visual><collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision><inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial></link></robot>'
    spawn_cube = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'cube', '-string', cube_urdf, '-x', '0.6', '-y', '0.0', '-z', '1.27'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        delayed_spawn,
        load_jsb,
        load_jtc,
        delayed_move_group,
        delayed_rviz,
        spawn_table,
        spawn_cube,
    ])

