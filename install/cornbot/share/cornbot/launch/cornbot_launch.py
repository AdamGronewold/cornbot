from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Absolute paths to your URDF and RViz configuration files
    urdf_file = '/src/cornbot/urdf/cornbot.urdf'
    rviz_config_file = '/src/cornbot/rviz/corbot_config.rviz'

    return LaunchDescription([
        # Node to start robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        # Node to start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])

