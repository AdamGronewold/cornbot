import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adamgronewold/ros2_ws/src/cornbot/install/cornbot'
