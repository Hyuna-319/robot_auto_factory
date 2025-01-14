from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch GUI node
        Node(
            package='user_gui',
            executable='user_gui',
            name='user_gui_node',
            output='screen',
        ),
        # Launch arUco detection node
        Node(
            package='user_gui',
            executable='aruco',
            name='aruco_node',
            output='screen',
        ),
        # Launch control node
        Node(
            package='user_gui',
            executable='control',
            name='control_node',
            output='screen',
        ),
        # Launch sorting node
        Node(
            package='user_gui',
            executable='sort',
            name='sort_node',
            output='screen',
        ),
        # Launch teleop test node
        Node(
            package='user_gui',
            executable='teleop_test',
            name='teleop_node',
            output='screen',
        ),
    ])

