import time
from rclpy.clock import Clock 

import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from matplotlib.colors import colorConverter

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from geometry_msgs.msg import QuaternionStamped
#__________________________________________________________________

class WaypointNode(Node):
    #------------------------------
    # Node and variable initialization
    def __init__(self):
        super().__init__('wps')
        self.sub1 = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.print_gnss_callback,1)
        self.sub2 = self.create_subscription(QuaternionStamped, '/cornbot/feeler_angles_topic', self.print_feeler_callback, 1)
        
    def print_gnss_callback(self, msg):
        print(msg.point.x)
        
    def print_feeler_callback(self,msg):
        print(msg.quaternion.x)
        
def main(args=None):
    rclpy.init(args=args)
    wps = WaypointNode()
    while 1:
        rclpy.spin_once(wps)


if __name__ == '__main__':
    main()
