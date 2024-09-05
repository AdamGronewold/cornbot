import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import QuaternionStamped

class FeelerNode(Node):
    def __init__(self):
        super().__init__('feeler_hub_node')
        self.get_logger().info("Initializing Feeler Node\n")
        self.ser = serial.Serial(port='/dev/ttyFeeler', baudrate=9600, timeout=0.01)
        self.get_logger().info("Feeler Serial On: %s\n" % self.ser.name)
        
        time.sleep(2)

        # Publishers for left and right state
        self.left_state_publisher = self.create_publisher(QuaternionStamped, 'feeler/left_state', 1)
        self.right_state_publisher = self.create_publisher(QuaternionStamped, 'feeler/right_state', 1)

        # Publisher for combined angles
        self.angles_publisher = self.create_publisher(QuaternionStamped, 'feeler/angles', 1)
                
        # Variables for storing previous values
        self.prev_left_angle = None
        self.prev_right_angle = None
        self.prev_left_vel = None
        self.prev_right_vel = None
        self.last_time = self.get_clock().now()

        timer_period = 0.008  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        try:
            new_data = self.ser.readline().decode('utf-8').strip()
            if new_data:
                data_parts = new_data.split(',')
                data_dict = {key.strip(): float(value.strip()) for key, value in (part.split('=') for part in data_parts)}

                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9

                left_angle = -1.0 * data_dict['BL']
                right_angle = -1.0 * data_dict['BR']

                # Calculate angular velocity (first derivatives)
                if self.prev_left_angle is not None and self.prev_right_angle is not None:
                    left_vel = (left_angle - self.prev_left_angle) / dt
                    right_vel = (right_angle - self.prev_right_angle) / dt

                    # Calculate angular acceleration (second derivatives)
                    if self.prev_left_vel is not None and self.prev_right_vel is not None:
                        left_acc = (left_vel - self.prev_left_vel) / dt
                        right_acc = (right_vel - self.prev_right_vel) / dt

                        # Publish left state (position, velocity, acceleration)
                        left_state_msg = QuaternionStamped()
                        left_state_msg.header.stamp = current_time.to_msg()
                        left_state_msg.header.frame_id = 'feeler_left_frame'
                        left_state_msg.quaternion.x = left_angle
                        left_state_msg.quaternion.y = left_vel
                        left_state_msg.quaternion.z = left_acc
                        left_state_msg.quaternion.w = 0.0
                        self.left_state_publisher.publish(left_state_msg)

                        # Publish right state (position, velocity, acceleration)
                        right_state_msg = QuaternionStamped()
                        right_state_msg.header.stamp = current_time.to_msg()
                        right_state_msg.header.frame_id = 'feeler_right_frame'
                        right_state_msg.quaternion.x = right_angle
                        right_state_msg.quaternion.y = right_vel
                        right_state_msg.quaternion.z = right_acc
                        right_state_msg.quaternion.w = 0.0
                        self.right_state_publisher.publish(right_state_msg)

                    self.prev_left_vel = left_vel
                    self.prev_right_vel = right_vel

                self.prev_left_angle = left_angle
                self.prev_right_angle = right_angle
                self.last_time = current_time

                # Create and publish quaternion message for angles
                angles_msg = QuaternionStamped()
                angles_msg.header.stamp = current_time.to_msg()
                angles_msg.header.frame_id = 'feeler_angles_frame'
                angles_msg.quaternion.x = left_angle
                angles_msg.quaternion.y = right_angle
                angles_msg.quaternion.z = 0.0
                angles_msg.quaternion.w = 0.0
                self.angles_publisher.publish(angles_msg)
                
        except KeyboardInterrupt:
            self.get_logger().fatal("\nKeyboard interrupt during serial read of feeler data. Killing node. Closing serial.\n")
            time.sleep(1)
            self.ser.close()
            time.sleep(1)
            self.destroy_node()
            time.sleep(1)
            exit()
        except UnicodeDecodeError:
            self.get_logger().error("Unicode decode error")
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
def main(args=None):
    rclpy.init(args=args)
    feeler_node = FeelerNode()
    feeler_node.ser.write(str.encode("<r>")) #"<Reset>"
    try:
        rclpy.spin(feeler_node)
    except KeyboardInterrupt:
        feeler_node.get_logger().fatal("Keyboard interrupt of ROS2 spin on feeler node. Killing node. Closing serial.")
        time.sleep(1)
        feeler_node.ser.close()
        time.sleep(1)
        feeler_node.destroy_node()
        time.sleep(1)
        rclpy.shutdown()
        exit()

if __name__ == "__main__":
    main()

