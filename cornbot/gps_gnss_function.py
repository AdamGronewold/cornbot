#make the ros node class available for inheritance by our custom node
import rclpy
import serial
import time
from pynmeagps import NMEAReader


from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
import uuid


class GNSSNode(Node): #Make a subclass of the Node class called "GNSSNode"

    def __init__(self):
        super().__init__('gnss_node') #call the constructor of the class from which this class is inhereted, providing the new node name as "gnss_node"
        
        #prior to starting the simulation you may need to unplug and replug in the GPS/GNSS  
        self.gnss_serial_messages=serial.Serial(port='/dev/ttyACM2', baudrate=115200, timeout=10) #this is the serial line that the GPS is sending the messages over
        
        self.my_NMEAReader=NMEAReader(self.gnss_serial_messages)
        
        #self.gnss_serial_config=serial.Serial(port='/dev/ttyUSB0', baudrate=115200) #this is the serial line used to configure the module
        if self.gnss_serial_messages.is_open !=True:
            while 1:
                self.get_logger().error('GNSS Serial Failure')

        self.get_logger().info("GNSS Serial On: %s" % self.gnss_serial_messages.name)
        
        #declares publisher publishes String messages on the "cornbot/gnss_nmea_stream_topic" topic, with a max queue size of 5
        self.NMEA_publisher = self.create_publisher(String, 'cornbot/gnss_nmea_string_stream_topic', 1) 
        self.NMEA_publisher2 = self.create_publisher(String, 'cornbot/gnss_nmea_raw_stream_topic', 1) 
        self.NMEA_publisher3 = self.create_publisher(String, 'cornbot/gnss_nmea_parsed_stream_topic', 1) 
        
        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        new_gnss_data=self.gnss_serial_messages.readline().decode('utf-8')
        msg = String()	
        msg.data = new_gnss_data
        self.NMEA_publisher.publish(msg) #publish the message
        self.get_logger().info('Publishing GNSS: "%s"' % msg.data) #inform ROS2 console as a ros info level message
    
        #(raw_data, parsed_data) = self.my_NMEAReader.read()
        #msg2=String()
        #msg2.data=str(raw_data)
        #self.NMEA_publisher2.publish(msg2)
        
        #msg3=String()
        #msg3.data=str(parsed_data)
        #self.NMEA_publisher3.publish(msg3)
        #self.get_logger().info("Parsed NMEA: %s" % parsed_data)


def main(args=None):

    rclpy.init(args=args) #initialize a ros2 instance
    gnss_node = GNSSNode() #initialize the node

    rclpy.spin(gnss_node) #set the node in motion

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gnss_node.gnss_serial_messages.close()
    gnss_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
