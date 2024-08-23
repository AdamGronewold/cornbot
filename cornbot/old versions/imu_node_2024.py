import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from rclpy.clock import Clock

import serial
import time

class IMUNode(Node): # Make a subclass of the Node class called "IMUNode"
    def __init__(self):
        super().__init__('imu_node') # Call constructor of the class this is inherited from with new name as imu_node
        self.get_logger().info("Initializing IMU Node\n")
        self.imu_serial = serial.Serial(port='/dev/ttyIMU', baudrate=115200, timeout=0.1) #/dev path is a shortcut to the associated usb device as set by udev rules under /etc/udev/rules.d/99-usb-serial.rules
        time.sleep(2)
        self.get_logger().info("IMU Serial On: %s\n" % self.imu_serial.name)
        
        # Declare publishers on 'cornbot/imu/...' topics
        self.raw_imu_pub = self.create_publisher(String, 'cornbot/imu/raw_topic', 1)
        self.last_stream_time_pub = self.create_publisher(Float32, 'cornbot/imu/last_stream_time_ms', 1)
        self.acc_x_pub = self.create_publisher(Float32, 'cornbot/imu/acc/x_mps2', 1)
        self.acc_y_pub = self.create_publisher(Float32, 'cornbot/imu/acc/y_mps2', 1)
        self.acc_z_pub = self.create_publisher(Float32, 'cornbot/imu/acc/z_mps2', 1)
        self.lin_acc_x_pub = self.create_publisher(Float32, 'cornbot/imu/lin_acc/x_mps2', 1)
        self.lin_acc_y_pub = self.create_publisher(Float32, 'cornbot/imu/lin_acc/y_mps2', 1)
        self.lin_acc_z_pub = self.create_publisher(Float32, 'cornbot/imu/lin_acc/z_mps2', 1)
        self.grav_acc_x_pub = self.create_publisher(Float32, 'cornbot/imu/grav_acc/x_mps2', 1)
        self.grav_acc_y_pub = self.create_publisher(Float32, 'cornbot/imu/grav_acc/y_mps2', 1)
        self.grav_acc_z_pub = self.create_publisher(Float32, 'cornbot/imu/grav_acc/z_mps2', 1)
        self.euler_heading_pub = self.create_publisher(Float32, 'cornbot/imu/euler/heading_deg', 1)
        self.euler_roll_pub = self.create_publisher(Float32, 'cornbot/imu/euler/roll_deg', 1)
        self.euler_pitch_pub = self.create_publisher(Float32, 'cornbot/imu/euler/pitch_deg', 1)
        self.calib_acc_pub = self.create_publisher(Int32, 'cornbot/imu/calib/acc', 1)
        self.calib_mag_pub = self.create_publisher(Int32, 'cornbot/imu/calib/mag', 1)
        self.calib_gyro_pub = self.create_publisher(Int32, 'cornbot/imu/calib/gyro', 1)
        self.calib_sys_pub = self.create_publisher(Int32, 'cornbot/imu/calib/sys', 1)
        
        # Additional publishers for combined data
        self.acc_pub = self.create_publisher(Vector3Stamped, 'cornbot/imu/acceleration_mps2', 1)
        self.lin_acc_pub = self.create_publisher(Vector3Stamped, 'cornbot/imu/linear_acceleration_mps2', 1)
        self.grav_acc_pub = self.create_publisher(Vector3Stamped, 'cornbot/imu/gravity_mps2', 1)
        self.euler_pub = self.create_publisher(Vector3Stamped, 'cornbot/imu/euler_angles_deg', 1)
        self.calib_pub = self.create_publisher(String, 'cornbot/imu/calibration_status', 1)
        
        timer_period = 0.01 # New message check period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        if self.imu_serial.inWaiting() > 0 and self.imu_serial.inWaiting() < 4095: # If serial input buffer isn't full
            try:
                imu_data_raw = self.imu_serial.readline().decode("utf-8")
                imu_raw_msg = String()
                imu_raw_msg.data = imu_data_raw
                self.raw_imu_pub.publish(imu_raw_msg)
                
                # Split the raw data into individual components
                imu_data = imu_data_raw.strip().split(',')
                
                # Parse and publish each part
                last_stream_time = float(imu_data[0].strip())
                acc_x = float(imu_data[1].strip())
                acc_y = float(imu_data[2].strip())
                acc_z = float(imu_data[3].strip())
                lin_acc_x = float(imu_data[4].strip())
                lin_acc_y = float(imu_data[5].strip())
                lin_acc_z = float(imu_data[6].strip())
                grav_acc_x = float(imu_data[7].strip())
                grav_acc_y = float(imu_data[8].strip())
                grav_acc_z = float(imu_data[9].strip())
                euler_heading = float(imu_data[10].strip())
                euler_roll = float(imu_data[11].strip())
                euler_pitch = float(imu_data[12].strip())
                calib_acc = int(imu_data[13].strip())
                calib_mag = int(imu_data[14].strip())
                calib_gyro = int(imu_data[15].strip())
                calib_sys = int(imu_data[16].strip())
                
                # Publish each part to its corresponding topic
                self.last_stream_time_pub.publish(Float32(data=last_stream_time))
                self.acc_x_pub.publish(Float32(data=acc_x))
                self.acc_y_pub.publish(Float32(data=acc_y))
                self.acc_z_pub.publish(Float32(data=acc_z))
                self.lin_acc_x_pub.publish(Float32(data=lin_acc_x))
                self.lin_acc_y_pub.publish(Float32(data=lin_acc_y))
                self.lin_acc_z_pub.publish(Float32(data=lin_acc_z))
                self.grav_acc_x_pub.publish(Float32(data=grav_acc_x))
                self.grav_acc_y_pub.publish(Float32(data=grav_acc_y))
                self.grav_acc_z_pub.publish(Float32(data=grav_acc_z))
                self.euler_heading_pub.publish(Float32(data=euler_heading))
                self.euler_roll_pub.publish(Float32(data=euler_roll))
                self.euler_pitch_pub.publish(Float32(data=euler_pitch))
                self.calib_acc_pub.publish(Int32(data=calib_acc))
                self.calib_mag_pub.publish(Int32(data=calib_mag))
                self.calib_gyro_pub.publish(Int32(data=calib_gyro))
                self.calib_sys_pub.publish(Int32(data=calib_sys))
                
                # Publish combined messages
                timestamp = Clock().now().to_msg()
                
                acc_msg = Vector3Stamped()
                acc_msg.header.stamp = timestamp
                acc_msg.header.frame_id = "/imu"
                acc_msg.vector.x = acc_x
                acc_msg.vector.y = acc_y
                acc_msg.vector.z = acc_z
                self.acc_pub.publish(acc_msg)
                
                lin_acc_msg = Vector3Stamped()
                lin_acc_msg.header.stamp = timestamp
                lin_acc_msg.header.frame_id = "/imu"
                lin_acc_msg.vector.x = lin_acc_x
                lin_acc_msg.vector.y = lin_acc_y
                lin_acc_msg.vector.z = lin_acc_z
                self.lin_acc_pub.publish(lin_acc_msg)
                
                grav_acc_msg = Vector3Stamped()
                grav_acc_msg.header.stamp = timestamp
                grav_acc_msg.header.frame_id = "/imu"
                grav_acc_msg.vector.x = grav_acc_x
                grav_acc_msg.vector.y = grav_acc_y
                grav_acc_msg.vector.z = grav_acc_z
                self.grav_acc_pub.publish(grav_acc_msg)
                
                euler_msg = Vector3Stamped()
                euler_msg.header.stamp = timestamp
                euler_msg.header.frame_id = "/imu"
                euler_msg.vector.x = euler_heading
                euler_msg.vector.y = euler_roll
                euler_msg.vector.z = euler_pitch
                self.euler_pub.publish(euler_msg)
                
                calib_msg = String()
                calib_msg.data = f"acc: {calib_acc}, mag: {calib_mag}, gyro: {calib_gyro}, sys: {calib_sys}"
                self.calib_pub.publish(calib_msg)
                
            except KeyboardInterrupt:
                self.get_logger().fatal("Keyboard interrupt during serial read of IMU data. Killing node. Closing serial.\n")
                self.imu_serial.flushInput()
                self.imu_serial.reset_input_buffer()
                self.imu_serial.close()
                time.sleep(2)
                exit()
            except UnicodeDecodeError:
                self.get_logger().error("Unicode decode error.\n")
            except Exception as e:
                self.get_logger().error("Error: %s" % e)
        elif self.imu_serial.inWaiting() > 4094:
            self.get_logger().warn("IMU serial input buffer overload. Flushing input buffer.\n") # Ensures we don't encounter errors on startup that occur from serial sentences partially sent by serial
            self.imu_serial.flushInput()
            self.imu_serial.reset_input_buffer()
            
def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode() # Initialize the imu node
    
    try:
        rclpy.spin(imu_node) # Try to spin the node but kill serial if...
    except KeyboardInterrupt:
        time.sleep(2)
        imu_node.get_logger().fatal("Keyboard interrupt of ROS2 spin of IMU node. Killing node. Closing serial.\n")
        imu_node.imu_serial.flushInput()
        imu_node.imu_serial.reset_input_buffer()
        imu_node.imu_serial.close()
        time.sleep(2)
        imu_node.destroy_node()
        rclpy.shutdown()
        time.sleep(2)
        exit()
        
if __name__ == "__main__":
    main()

