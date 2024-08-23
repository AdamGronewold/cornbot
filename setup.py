from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cornbot'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Including the URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Gronewold',
    maintainer_email='Adam.M.Gronewold.TH@Dartmouth.edu',
    description='This package is for the ROS infrastructure needed to navigate a small robot through a cornfield with an array of sensors',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodes made in 2023
            'motor_control_node = cornbot.motor_control_function:main',
            'xbox_teleop_node = cornbot.xbox_teleop:main',
            #'gnss_node = cornbot.gps_gnss_function:main',
            #'sensor_node = cornbot.sensor_hub_function:main', #no longer works with current hardware
            #'daq_node = cornbot.daq_control_function:main',
            
            # Nodes made in 2024
            'feeler_node2 = cornbot.feeler_function_2024:main',
            'gnss_node2 = cornbot.gps_gnss_function_2024:main',
            'motor_control_node2 = cornbot.motor_control_function_2024:main',
            'joint_state_node = cornbot.joint_state_publisher:main',
            'odom_tf_broadcaster = cornbot.odom_tf_broadcaster:main',
            'odom_tf_broadcaster2 = cornbot.odom_tf_broadcaster2:main',
            'feeler_state_node = cornbot.feeler_contact_state_id:main',
            #'feeler_localization = cornbot.feeler_localization:main',
            #'feeler_plot = cornbot.feeler_state_plotter:main',
            #'imu_node2 = cornbot.imu_node_2024:main',
            #'init_heading_node = cornbot.initialization_sequence_2024:main',
            #'get_waypoint_node = cornbot.set_waypoint:main', 
            #'waypoint_nav = cornbot.waypoint_nav2:main',
            #'waypoint_nav2 = cornbot.waypoint_nav2_1:main',
            #'feeler_control = cornbot.feeler_controller_dev:main',
            #'with_feelers = cornbot.waypoint_nav_with_feelers1:main',
            #'sub_test = cornbot.subscriber_test:main',
            #'with_feelers2 = cornbot.pure_pursuit_with_feelers:main',
            #'simple_feeler_data = cornbot.simple_feeler_data_node_July_25_2024:main',
        ],
    },
)

