import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(pkg_name, file_path):
    pkg_path = get_package_share_directory(pkg_name)
    abs_path = os.path.join(pkg_path, file_path)
    with open(abs_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():

    pkg_path = get_package_share_directory('ur5_description')

    # ── Robot description (URDF) ─────────────────────────────────────────────
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur5.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ── Semantic description (SRDF) ──────────────────────────────────────────
    srdf_file = os.path.join(pkg_path, 'config', 'ur5.srdf')
    with open(srdf_file) as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ── Kinematics ───────────────────────────────────────────────────────────
    kinematics_yaml = load_yaml('ur5_description', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # ── Joint limits ─────────────────────────────────────────────────────────
    joint_limits_yaml = load_yaml('ur5_description', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # ── Planning pipeline (OMPL) ─────────────────────────────────────────────
    ompl_yaml = load_yaml('ur5_description', 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,
    }

    # ── Trajectory execution ─────────────────────────────────────────────────
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # ── MoveIt controllers ───────────────────────────────────────────────────
    moveit_controllers_yaml = load_yaml('ur5_description', 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager':
            moveit_controllers_yaml['moveit_simple_controller_manager'],
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # ── move_group node ──────────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            {'use_sim_time': True},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
        ],
    )

    # ── RViz2 with MoveIt plugin ─────────────────────────────────────────────
    rviz_config = os.path.join(pkg_path, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
