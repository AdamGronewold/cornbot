from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
  
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
                    executable='feeler_state_node',
                    name='feeler_contact_id',
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cornbot',
                    executable='feeler_localization',
                    name='feeler_localization',
                    output='screen'
                ),
            ]
        )
    ])

