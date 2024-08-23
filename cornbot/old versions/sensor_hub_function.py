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
#Name a subclass of Class:Node named "SensorHubNode"
class SensorHubNode(Node):


    #SpeedRefSubscriber constructor
    def __init__(self):
    
        self.arduino_uno = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=0.01)
    	
    	#call our super class "Node" and its constructor, 
    	#passing it the name of our new node as "sensor_hub_node'
        super().__init__('sensor_hub_node') 
                
        self.publisher1 = self.create_publisher(Vector3, 'cornbot/IMU/euler_angles_topic', 1)
        self.publisher2 = self.create_publisher(Vector3, 'cornbot/IMU/accelerometer_topic', 1)
        self.publisher3 = self.create_publisher(Vector3, 'cornbot/IMU/linear_accel_topic', 1)
        self.publisher4 = self.create_publisher(Vector3, 'cornbot/IMU/grav_accel_topic',1)
        self.publisher5 = self.create_publisher(Vector3, 'cornbot/feeler_angles_topic', 1)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.pub_wheel_speeds_callback)

	
    def pub_wheel_speeds_callback(self):
        try:
            new_data=self.arduino_uno.readline().decode('utf-8')
        except UnicodeDecodeError:
            new_data=self.arduino_uno.readline().decode('utf-8')
        #self.get_logger().info('Publishing data')       
        data_split = new_data.split(', ')
        if data_split[0]=='':
            pass
        else:
            time=data_split[0] #time in ms
            
            msg1=Vector3()
            msg1.x=float(data_split[1]) #heading deg
            msg1.y=float(data_split[2]) #roll deg
            msg1.z=float(data_split[3]) #pitch deg
            self.publisher1.publish(msg1)
            
            msg2=Vector3()
            msg2.x=float(data_split[4]) #aX m/s2
            msg2.y=float(data_split[5]) #aY m/s2
            msg2.z=float(data_split[6])#aZ m/s2
            self.publisher2.publish(msg2)
            
            msg3=Vector3()
            msg3.x=float(data_split[7])#m/s2
            msg3.y=float(data_split[8])#m/s2
            msg3.z=float(data_split[9])#m/s2
            self.publisher3.publish(msg3)
            
            msg4=Vector3()
            msg4.x=float(data_split[10])#m/s2
            msg4.y=float(data_split[11])#m/s2
            msg4.z=float(data_split[12])#m/s2
            self.publisher4.publish(msg4)
            
            msg5=Vector3()
            msg5.x=float(data_split[17]) #right feeler deg
            msg5.y=float(data_split[18]) #left feelerdeg
            msg5.z=float(data_split[19]) #NA
            self.publisher5.publish(msg5)
            
            #these can also be published if needed
            A=data_split[13] #acceleration calibration value (0-3)
            M=data_split[14] #magnetometer calibration value (0-3)
            G=data_split[15] #gyroscope calibration value (0-3)
            S=data_split[16] #system calibration value (0-3)


def main(args=None):
       
    rclpy.init(args=args) #initialize ROS2 instance

    sensor_hub_node = SensorHubNode() #initialize our subscriber

    rclpy.spin(sensor_hub_node) #spin our motor controller node
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_hub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
