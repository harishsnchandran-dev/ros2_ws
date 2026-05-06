import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg_share = FindPackageShare('parker_actuator')

    # ── Launch Arguments ──────────────────────────────────────────────────────
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='true',   # ← default ON: safe to run without hardware
        description=(
            'Set to "true" to use mock hardware (no physical actuator needed). '
            'Set to "false" to connect to real Parker Compax3 via RS232.'
        )
    )
    sim_mode = LaunchConfiguration('sim_mode')

    # ── Robot Description (URDF from Xacro) ──────────────────────────────────
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'parker_actuator.xacro']),
            ' sim_mode:=', sim_mode,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Semantic Description (SRDF) ───────────────────────────────────────────
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='cat')]),
            ' ',
            PathJoinSubstitution([pkg_share, 'config', 'parker_actuator.srdf']),
        ]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # ── MoveIt Config ─────────────────────────────────────────────────────────
    kinematics_yaml         = load_yaml('parker_actuator', 'config/kinematics.yaml')
    joint_limits_yaml       = load_yaml('parker_actuator', 'config/joint_limits.yaml')
    ompl_planning_yaml      = load_yaml('parker_actuator', 'config/ompl_planning.yaml')
    moveit_controllers_yaml = load_yaml('parker_actuator', 'config/moveit_controllers.yaml')

    planning_pipelines_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_yaml,
    }

    move_group_capabilities = {
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ── Controller Config ─────────────────────────────────────────────────────
    robot_controllers = PathJoinSubstitution(
        [pkg_share, 'config', 'parker_actuator_controllers.yaml']
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # ros2_control node — loads hardware interface
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )

    # Robot state publisher — TF from URDF
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Spawners — wait for controller_manager
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    parker_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'parker_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    # MoveGroup — motion planning server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            planning_pipelines_config,
            move_group_capabilities,
            moveit_controllers_yaml,   # ← tells MoveIt about parker_controller
        ],
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription([
        sim_mode_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        parker_controller_spawner,
        move_group_node,
        rviz_node,
    ])
