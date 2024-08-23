from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    urdf_file_path = '/home/adamgronewold/ros2_ws/src/cornbot/urdf/cornbot.urdf'
    rviz_config_path = '/home/adamgronewold/ros2_ws/src/cornbot/rviz/cornbot_config.rviz'
    
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    return LaunchDescription([

        Node(
            package='cornbot',
            executable='feeler_node2',
            name='feeler_sensors',
            output='screen',
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='motor_control_node2',
                    name='motor_control',
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='gnss_node2',
                    name='gnss_node',
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='joint_state_node',
                    name='joint_state_publisher',
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
		    package='cornbot',
		    executable='odom_tf_broadcaster', 
		    name='odom_tf_broadcaster',       
		    output='screen',
		),
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': urdf_content}],
                ),
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],
                ),
            ]
        ),

    ])

