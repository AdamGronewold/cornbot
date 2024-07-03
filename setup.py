from setuptools import setup
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
            #Nodes made in 2023
            'motor_control_node = cornbot.motor_control_function:main',
            'xbox_teleop_node = cornbot.xbox_teleop:main',
            'gnss_node = cornbot.gps_gnss_function:main',
            'sensor_node = cornbot.sensor_hub_function:main', #no longer works with current hardware
            'daq_node = cornbot.daq_control_function:main',
            
            #Nodes made in 2024
            'feeler_node2 = cornbot.feeler_function_2024:main',
            'gnss_node2 = cornbot.gps_gnss_function_2024:main',
            'motor_control_node2 = cornbot.motor_control_function_2024:main',
            'imu_node2 = cornbot.imu_node_2024:main',
            'serial_test = cornbot.serial_test',
            'init_heading_node = cornbot.initialization_sequence_2024:main',
            'get_waypoint_node = cornbot.set_waypoint:main', 
            'waypoint_nav = cornbot.waypoint_nav1:main',
        ],
    },
)
