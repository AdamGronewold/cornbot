from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import ament_index_python
from launch.substitutions import LaunchConfiguration
import yaml
import os

def generate_launch_description():
    # Get the package share directory for cornbot
    package_share_directory = ament_index_python.packages.get_package_share_directory('cornbot')
    urdf_file_path = '/home/adamgronewold/ros2_ws/src/cornbot/urdf/cornbot.urdf'
    rviz_config_path = '/home/adamgronewold/ros2_ws/src/cornbot/rviz/cornbot_config.rviz'

    # Path to the launch files and directories that we will use
    _MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
    _FRAMES_PARAMS_FILE ='/home/adamgronewold/ros2_ws/src/cornbot/config/params.yml'
    _PACKAGE_NAME = 'microstrain_inertial_driver'
    _DEFAULT_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory(_PACKAGE_NAME), 'microstrain_inertial_driver_common', 'config', 'params.yml')
	
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    return LaunchDescription([

        Node(
            package='cornbot',
            executable='feeler_node2',
            name='feeler_sensors',
            namespace='cornbot',
            output='screen',
        ),

        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='motor_control_node2',
                    name='motor_control',
                    namespace='cornbot',
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
                    namespace='cornbot',
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='joint_state_node',
                    name='joint_state_publisher',
                    namespace='cornbot',
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='odom_tf_broadcaster2', 
                    name='odom_tf_broadcaster',   
                    namespace='cornbot',    
                    output='screen',
                ),
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    namespace='cornbot',
                    output='screen',
                    parameters=[{'robot_description': urdf_content}],
                ),
            ]
        ),      

        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='feeler_state_node',
                    name='feeler_contact_id',
                    namespace='cornbot',
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='feeler_localization',
                    name='feeler_localization',
                    namespace='cornbot',
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=20.0,
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
        
        TimerAction(
            period=1.0,
            actions=[    	
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
                        launch_arguments={
                            'configure': 'true',
                            'activate': 'true',
                            'params_file': _FRAMES_PARAMS_FILE,
                            'namespace': '/cornbot',
                        }.items()
                ),
            ]
        )        
    ])

