from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='agv_hand_control',
            executable='mediapipe_node',
            name='mediapipe_node',
            output='screen'
        ),

        Node(
            package='agv_obstacle_detection',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),

        Node(
            package='agv_motion_manager',
            executable='motion_manager',
            name='motion_manager',
            output='screen'
        ),

        Node(
            package='agv_ui',
            executable='ui_node',
            name='ui_node',
            output='screen'
        ),

    ])