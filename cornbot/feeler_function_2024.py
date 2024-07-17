import serial
import time

# Import ROS and ROS Node infrastructure
import rclpy
from rclpy.node import Node

# Used for message passing
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import QuaternionStamped

# Name a subclass of Class:Node named "FeelerHubNode"
class FeelerNode(Node):
    # Feeler node constructor
    def __init__(self):
        # Call our super class "Node" and its constructor,
        # passing it the name of our new node as "feeler_hub_node"
        super().__init__('feeler_hub_node')
        self.get_logger().info("Initializing Feeler Node\n")
        self.ser = serial.Serial(port='/dev/ttyFeeler', baudrate=9600, timeout=0.01) #/dev path is a shortcut to the associated usb device as set by udev rules under /etc/udev/rules.d/99-usb-serial.rules
        self.get_logger().info("Feeler Serial On: %s\n" % self.ser.name)
        
        time.sleep(2)
        self.publisher = self.create_publisher(QuaternionStamped, 'cornbot/feeler_angles_topic', 1)
        self.front_left_publisher = self.create_publisher(Float32, 'cornbot/feeler_angles_topic/front_left', 1)
        self.front_right_publisher = self.create_publisher(Float32, 'cornbot/feeler_angles_topic/front_right', 1)
        self.back_left_publisher = self.create_publisher(Float32, 'cornbot/feeler_angles_topic/back_left', 1)
        self.back_right_publisher = self.create_publisher(Float32, 'cornbot/feeler_angles_topic/back_right', 1)
        
        timer_period = 0.008  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) #check periodically for incoming data
        
        # timer_period2 = 5 # second
        # self.reset_timer=self.create_timer(timer_period2, self.reset_callback)
        
    def timer_callback(self):
        
        try:
            new_data = self.ser.readline().decode('utf-8').strip()
            if new_data:
                #self.get_logger().info(f"Received data: {new_data}")
                # Parse the data
                data_parts = new_data.split(',')
                data_dict = {}
                for part in data_parts:
                    key, value = part.split('=')
                    data_dict[key.strip()] = float(value.strip())
                
                # Create and publish quaternion message
                quaternion_msg = QuaternionStamped()
                quaternion_msg.header.stamp = self.get_clock().now().to_msg()
                quaternion_msg.header.frame_id = 'feeler_frame'
                quaternion_msg.quaternion.x = data_dict['FL']
                quaternion_msg.quaternion.y = data_dict['FR']
                quaternion_msg.quaternion.z = data_dict['BL']
                quaternion_msg.quaternion.w = data_dict['BR']
                self.publisher.publish(quaternion_msg)

                # Publish individual float messages
                self.front_left_publisher.publish(Float32(data=data_dict['FL']))
                self.front_right_publisher.publish(Float32(data=data_dict['FR']))
                self.back_left_publisher.publish(Float32(data=data_dict['BL']))
                self.back_right_publisher.publish(Float32(data=data_dict['BR']))
                
        except KeyboardInterrupt:
            time.sleep(2)
            self.get_logger().fatal("\nKeyboard interrupt during serial read of feeler data. Killing node. Closing serial.\n")
            time.sleep(2)
            self.ser.close()
            exit()
        except UnicodeDecodeError:
            self.get_logger().error("Unicode decode error")
        except Exception as e:
            self.get_logger().error('Error: %s' % e)
            
    # def reset_callback(self):
    #    try:
    #        self.ser.write(str.encode("<r>")) #or "<Reset>"
    #    except Exception as e:
    #        self.get_logger().error('Error on reset: %s' % e)
            
def main(args=None):

    rclpy.init(args=args) #initialize a ros2 instance
    feeler_node = FeelerNode() #initialize the node
    feeler_node.ser.write(str.encode("<r>")) #"<Reset>"
    try:
        rclpy.spin(feeler_node) #set the node in motion
    except KeyboardInterrupt:
        time.sleep(2)
        feeler_node.get_logger().fatal("Keyboard interrupt of ROS2 spin on feeler node. Killing node. Closing serial.")
        feeler_node.ser.close()
        feeler_node.destroy_node()
        rclpy.shutdown()
        time.sleep(2)
        exit()
            
if __name__ == "__main__":
    main()

