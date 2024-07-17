import rclpy
import time
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node

class FeelerController(Node):
	def __init__(self):
		super().__init__('feeler_control_node')
		time.sleep(1)
		
		self.sub=self.create_subscription(QuaternionStamped, '/cornbot/feeler_angles_topic', self.update_angles_callback, 1)
		self.FL=0
		self.FR=0
		self.BL=0
		self.BR=0
		
	def update_angles_callback(self, msg):
		quaternion_msg = msg

		self.FL=quaternion_msg.quaternion.x
		print(self.FL)
	

def main(args=None):

	rclpy.init(args=args) #initialize a ros2 instance
	feeler_controller = FeelerController() #initialize the node

	try:
		rclpy.spin(feeler_controller) #set the node in motion
	except KeyboardInterrupt:
		time.sleep(2)
		feeler_controller.get_logger().fatal("Keyboard interrupt of ROS2 spin on feeler controller. Killing node.")
		feeler_controller.ser.close()
		feeler_controller.destroy_node()
		rclpy.shutdown()
		time.sleep(2)
		exit()
            
if __name__ == "__main__":
    main()

