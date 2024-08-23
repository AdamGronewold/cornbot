import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=0.0,  # Launch immediately
            actions=[
                Node(
                    package='cornbot',
                    executable='gnss_node2',
                    name='gnss_node2',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=5.0,  # Launch 5 seconds after the first node
            actions=[
                Node(
                    package='cornbot',
                    executable='motor_control_node2',
                    name='motor_control_node2',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=10.0,  # Launch 5 seconds after the second node
            actions=[
                Node(
                    package='cornbot',
                    executable='xbox_teleop_node',
                    name='xbox_teleop_node',
                    output='screen'
                )
            ]
        ),
    ])

