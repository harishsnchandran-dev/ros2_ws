import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    pkg_name = 'quadbot'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Force processing the SOURCE xacro for development speed
    source_xacro = '/home/harish/ros2_ws/src/quadbot/urdf/quadbot.xacro'
    controllers_yaml = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    robot_description_config = xacro.process_file(
        source_xacro,
        mappings={'controllers_yaml': controllers_yaml}
    )
    from launch_ros.parameter_descriptions import ParameterValue
    
    # Aggressive XML scrub for ROS 2 Humble
    raw_xml = robot_description_config.toxml()
    # Find the start of the <robot tag and keep everything after it
    start_idx = raw_xml.find('<robot')
    if start_idx != -1:
        robot_description_xml = raw_xml[start_idx:]
    else:
        robot_description_xml = raw_xml
        
    robot_description = {'robot_description': ParameterValue(robot_description_xml, value_type=str)}
    
    # Gazebo Path
    world_path = os.path.join(pkg_dir, 'worlds', 'rough_terrain.world')
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    
    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'quadbot', '-z', '0.3'],
        output='screen'
    )
    
    # Controllers setup
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_group_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_group_position_controller'],
        output='screen'
    )
    
    # Gait Controller: converts /cmd_vel to joint positions
    gait_controller_node = Node(
        package=pkg_name,
        executable='gait_controller.py',
        output='screen'
    )

    # Autonomous Controller: uses LIDAR to publish /cmd_vel autonomously
    autonomous_controller_node = Node(
        package=pkg_name,
        executable='autonomous_controller.py',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_group_position_controller],
            )
        ),
        gait_controller_node,
        autonomous_controller_node,
    ])
