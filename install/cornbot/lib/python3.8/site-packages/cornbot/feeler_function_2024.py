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

        # Original publishers for angles
        self.publisher = self.create_publisher(QuaternionStamped, 'cornbot/feeler/all/angles', 1)
        self.front_left_publisher = self.create_publisher(Float32, 'cornbot/feeler/left/angles', 1)
        self.front_right_publisher = self.create_publisher(Float32, 'cornbot/feeler/right/angles', 1)

        # New publishers for angular velocities and accelerations (Float32)
        self.left_vel_pub = self.create_publisher(Float32, 'cornbot/feeler/left_vel', 1)
        self.right_vel_pub = self.create_publisher(Float32, 'cornbot/feeler/right_vel', 1)
        self.left_acc_pub = self.create_publisher(Float32, 'cornbot/feeler/left_acc', 1)
        self.right_acc_pub = self.create_publisher(Float32, 'cornbot/feeler/right_acc', 1)

        # New publishers for angular velocities and accelerations (QuaternionStamped)
        self.ang_vel_pub = self.create_publisher(QuaternionStamped, 'cornbot/feeler/all/ang_vel', 1)
        self.ang_acc_pub = self.create_publisher(QuaternionStamped, 'cornbot/feeler/all/ang_acc', 1)
                
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

                    # Publish angular velocities (Float32)
                    self.left_vel_pub.publish(Float32(data=left_vel))
                    self.right_vel_pub.publish(Float32(data=right_vel))

                    # Publish angular velocity quaternion message
                    ang_vel_msg = QuaternionStamped()
                    ang_vel_msg.header.stamp = current_time.to_msg()
                    ang_vel_msg.header.frame_id = 'feeler_frame'
                    ang_vel_msg.quaternion.x = left_vel
                    ang_vel_msg.quaternion.y = right_vel
                    ang_vel_msg.quaternion.z = 0.0
                    ang_vel_msg.quaternion.w = 0.0
                    self.ang_vel_pub.publish(ang_vel_msg)

                    # Calculate angular acceleration (second derivatives)
                    if self.prev_left_vel is not None and self.prev_right_vel is not None:
                        left_acc = (left_vel - self.prev_left_vel) / dt
                        right_acc = (right_vel - self.prev_right_vel) / dt

                        # Publish angular accelerations (Float32)
                        self.left_acc_pub.publish(Float32(data=left_acc))
                        self.right_acc_pub.publish(Float32(data=right_acc))

                        # Publish angular acceleration quaternion message
                        ang_acc_msg = QuaternionStamped()
                        ang_acc_msg.header.stamp = current_time.to_msg()
                        ang_acc_msg.header.frame_id = 'feeler_frame'
                        ang_acc_msg.quaternion.x = left_acc
                        ang_acc_msg.quaternion.y = right_acc
                        ang_acc_msg.quaternion.z = 0.0
                        ang_acc_msg.quaternion.w = 0.0
                        self.ang_acc_pub.publish(ang_acc_msg)

                    self.prev_left_vel = left_vel
                    self.prev_right_vel = right_vel

                self.prev_left_angle = left_angle
                self.prev_right_angle = right_angle
                self.last_time = current_time

                # Create and publish quaternion message for angles
                quaternion_msg = QuaternionStamped()
                quaternion_msg.header.stamp = current_time.to_msg()
                quaternion_msg.header.frame_id = 'feeler_frame'
                quaternion_msg.quaternion.x = left_angle
                quaternion_msg.quaternion.y = right_angle
                quaternion_msg.quaternion.z = 0.0
                quaternion_msg.quaternion.w = 0.0
                self.publisher.publish(quaternion_msg)

                # Publish individual float messages for angles
                self.front_left_publisher.publish(Float32(data=left_angle))
                self.front_right_publisher.publish(Float32(data=right_angle))
                
        except KeyboardInterrupt:
            self.get_logger().fatal("\nKeyboard interrupt during serial read of feeler data. Killing node. Closing serial.\n")
            self.ser.close()
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
        feeler_node.ser.close()
        feeler_node.destroy_node()
        rclpy.shutdown()
        exit()

if __name__ == "__main__":
    main()

