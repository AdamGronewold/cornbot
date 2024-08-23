#Import ROS and ROS Node infrastucture
import rclpy
from rclpy.node import Node

import serial
import time
#Used for message passing
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

#Name a subclass of Class:Node named "MotorControlNode"
class MotorControlNode(Node):


    #SpeedRefSubscriber constructor
    def __init__(self):
    
        self.teensy = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=0.01)
    	
    	#call our super class "Node" and its constructor, 
    	#passing it the name of our new node as "motor_control_node"
        super().__init__('motor_control_node') 
        
        #create the subscription to the "topic" topic watching for String messages, 
        #with 10 message buffer and callback to our subscriber callback function named "listener_callback"
        self.subscription1 = self.create_subscription(String, 'cornbot/speed_ref_topic', self.send_speed_ref_callback, 5)
        
        #create publishers to provide ROS system with knowledge of the robots wheel speeds
        #with callback timer to read data from teensy
        self.publisher1 = self.create_publisher(Float32, 'cornbot/wheel_rpm_left', 1)
        self.publisher2 = self.create_publisher(Float32, 'cornbot/wheel_rpm_right', 1)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.pub_wheel_speeds_callback)
        
        self.get_logger().info('Motor control node started')

    def send_speed_ref_callback(self, msg):
        self.get_logger().info('New reference speed (m/s) set (-right, -left): "%s"' % msg.data) #print to ROS console
        self.teensy.write(bytes(msg.data, 'utf-8'))
        time.sleep(0.1)
	#data=teensy.readline().decode('utf-8')
	#return data
	
    def pub_wheel_speeds_callback(self):
        try:
            new_data=self.teensy.readline().decode('utf-8')
        except UnicodeDecodeError:
            new_data=self.teensy.readline().decode('utf-8')
        
        self.get_logger().info("Wheel rpms: %s" % new_data)
        
        data_split = new_data.split(', ')
        msg1=Float32() #right rpm
        msg2=Float32() #right rpm
        msg3=Float32MultiArray() #all rpms

        if len(data_split)==2:
            try:
                left_rpm=float(data_split[0])
                right_rpm=float(data_split[1])
                
                msg1.data=left_rpm #left
                msg2.data=right_rpm  #right
                
                self.publisher1.publish(msg1) #publish the message
                self.publisher2.publish(msg2) #publish the message
                #self.get_logger().info('Publishing last wheel rpm (left, right): "%s"' % new_data) #inform ROS2 console as a ros info level message
            except:
                self.get_logger().info('Could not convert %s to float' % data_split[0])
                self.get_logger().info('Could not convert %s to float' % data_split[1])

def main(args=None):
    
    #teensy = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=0.01)
    
    rclpy.init(args=args) #initialize ROS2 instance

    motor_control_node = MotorControlNode() #initialize our subscriber

    rclpy.spin(motor_control_node) #spin our motor controller node
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
