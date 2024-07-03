from launch import LaunchDescription
from launch import actions
from launch_ros.actions import Node
import time

print("\n\n-------------------\nStarting Corn Robot\n------------------\n\n")
time.sleep(1)

def generate_launch_description():
    return LaunchDescription([
        actions.DeclareLaunchArgument(
            "log_level",
            default_value=["warn"],
            description="Logging level",
       ),
        Node(
            package='cornbot',
            #namespace='cornbot1',
            executable='xbox_teleop_node',
            #name='sim'
        ),
        Node(
            package='cornbot',
            #namespace='cornbot1',
            executable='motor_control_node',
            #name='sim'
        ),
        Node(
            package='cornbot',
            #namespace='cornbot1',
            executable='gnss_node',
            #name='sim'
        ),
    ])
