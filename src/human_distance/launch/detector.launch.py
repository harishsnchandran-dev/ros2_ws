from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera'
        ),

        Node(
            package='human_distance',
            executable='detector',
            name='human_distance_node'
        )

    ])