#Import ROS and ROS Node infrastucture
import rclpy
from rclpy.node import Node

import serial
import time

#Used for message passing
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import QuaternionStamped


#Name a subclass of Class:Node named "FeelerHubNode"
class FeelerHubNode(Node):


    #Feeler node constructor
    def __init__(self):
      	
    	#call our super class "Node" and its constructor, 
    	#passing it the name of our new node as "feeler_hub_node'
        super().__init__('feeler_hub_node') 
        
        self.arduino_due = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=0.01)
        
        self.publisher = self.create_publisher(QuaternionStamped, 'cornbot/feeler_angles_topic', 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.feeler_sensor_callback)

	
    def feeler_sensor_callback(self):
        
        try:
            new_data=self.arduino_due.readline()
            print(new_data)
            print('here')
        except UnicodeDecodeError:
            new_data=self.arduino_due.readline()
            print('there')


def main(args=None):
       
    rclpy.init(args=args) #initialize ROS2 instance
    
    feeler_hub_node = FeelerHubNode() #initialize our subscriber
    try:
    	rclpy.spin(feeler_hub_node) #spin our motor controller node
    finally:
        feeler_hub_node.arduino_due.close()
        
        time.sleep(2)
        print('serial closed')
        
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        feeler_hub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
