import rclpy
import time
import csv
from geometry_msgs.msg import QuaternionStamped, PointStamped
from rclpy.node import Node
from std_msgs.msg import String

class SimpleFeelerData(Node):
    def __init__(self):
        super().__init__('simple_feeler_data_node')
        time.sleep(1)
        
        # Subscriptions
        self.feeler_sub = self.create_subscription(QuaternionStamped, '/cornbot/feeler_angles_topic', self.update_angles_callback, 1)
        self.lat_lon_sub = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.update_position_callback, 1)
        self.course_sub = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/course_over_ground', self.update_heading_callback, 1)
        
        # Publisher
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        
        self.FL = 0
        self.FR = 0
        self.BL = 0
        self.BR = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.course_over_ground = 0.0
        
        self.log_file_path = 'src/cornbot/log/July25_2024_StraightTravelFeelerData0.2meterspersecond/SimpleFeelerData.csv'
        
        # Create a new CSV file and write the header
        with open(self.log_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'FL', 'FR', 'BL', 'BR', 'Longitude', 'Latitude', 'Course_Over_Ground'])

        # Publish the reference speed command
        self.publish_speed("<-0.2, -0.2>")

    def update_angles_callback(self, msg):
        quaternion_msg = msg
        self.FL = quaternion_msg.quaternion.x
        self.FR = quaternion_msg.quaternion.y
        self.BL = quaternion_msg.quaternion.z
        self.BR = quaternion_msg.quaternion.w
        
        self.log_data()

    def update_position_callback(self, msg):
        self.latitude = msg.point.x
        self.longitude = msg.point.y
        
        self.log_data()

    def update_heading_callback(self, msg):
        self.course_over_ground = msg.point.z
        
        self.log_data()

    def log_data(self):
        current_time = time.time()
        with open(self.log_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, self.FL, self.FR, self.BL, self.BR, self.longitude, self.latitude, self.course_over_ground])

    def publish_speed(self, speed):
        speed_msg = String()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    simple_feeler_data = SimpleFeelerData()

    try:
        rclpy.spin(simple_feeler_data)
    except KeyboardInterrupt:
        time.sleep(2)
        simple_feeler_data.get_logger().fatal("Keyboard interrupt of ROS2 spin on simple feeler data. Killing node.")
    except Exception as e:
        simple_feeler_data.get_logger().fatal(f"Exception in ROS2 spin: {e}")
        simple_feeler_data.publish_speed("<0, 0>")
        time.sleep(2)
    finally:
        simple_feeler_data.destroy_node()
        rclpy.shutdown()
        time.sleep(2)
        exit()

if __name__ == "__main__":
    main()

