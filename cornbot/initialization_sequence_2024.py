import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import math

class GpsSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            PointStamped,
            'cornbot/gnss/positioning/lat_lon_stamped_topic',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.current_lat = None
        self.current_lon = None
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        self.heading_pub = self.create_publisher(String, 'cornbot/gnss_track_heading_topic', 1)
        self.heading_estimates = []

    def listener_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y
        #self.get_logger().info(f'Received latitude: {self.current_lat}, longitude: {self.current_lon}')

    def publish_speed(self, right_ref, left_ref):
        speed_command = f'<{right_ref}, {left_ref}>'
        msg = String()
        msg.data = speed_command
        self.speed_pub.publish(msg)
        #self.get_logger().info(f'Published speed: {speed_command}')

    def average_position(self, duration=5, rate=4):
        start_time = time.time()
        latitudes = []
        longitudes = []
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=1 / rate)
            if self.current_lat is not None and self.current_lon is not None:
                latitudes.append(self.current_lat)
                longitudes.append(self.current_lon)
            time.sleep(1 / rate)
        self.get_logger().info(f'Collected {len(latitudes)} latitudes and {len(longitudes)} longitudes')
        if len(latitudes) > 0 and len(longitudes) > 0:
            avg_lat = sum(latitudes) / len(latitudes)
            avg_lon = sum(longitudes) / len(longitudes)
            self.get_logger().info(f'Average Latitude: {avg_lat}, Average Longitude: {avg_lon}')
            return avg_lat, avg_lon
        else:
            self.get_logger().error('No data collected for averaging')
            return None, None

    def calculate_heading(self, start_lat, start_lon, end_lat, end_lon):
        delta_lon = end_lon - start_lon
        delta_lat = end_lat - start_lat
        heading = math.degrees(math.atan2(delta_lon, delta_lat))
        self.get_logger().info(f'Calculated heading: {heading} degrees')
        return heading

    def initialization_maneuver(self):
        while True:
            # Start
            self.get_logger().info("Starting initialization...")
           
            # Capture initial GPS data
            self.get_logger().info('Capturing initial GPS data...')
            start_lat, start_lon = self.average_position()
            if start_lat is None or start_lon is None:
                self.get_logger().error('Failed to get initial GPS data.')
                continue

            # Move forward
            self.get_logger().info('Moving forward...')
            self.publish_speed(-0.2, -0.2)
            for _ in range(100):  # 25 iterations * 0.1s = 2.5 seconds
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            # Pause and capture GPS data
            self.get_logger().info('Pausing...')
            self.publish_speed(0.0, 0.0)

            self.get_logger().info('Capturing GPS data after moving forward...')
            mid_lat, mid_lon = self.average_position()
            if mid_lat is None or mid_lon is None:
                self.get_logger().error('Failed to get GPS data after moving forward.')
                continue

            # Calculate heading from initial to mid position
            heading1 = self.calculate_heading(start_lat, start_lon, mid_lat, mid_lon)

            # Move backward
            self.get_logger().info('Moving backward...')
            self.publish_speed(0.2, 0.2)
            for _ in range(100):  # 25 iterations * 0.1s = 2.5 seconds
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            self.publish_speed(0.0, 0.0)

            # Capture final GPS data
            self.get_logger().info('Capturing final GPS data...')
            end_lat, end_lon = self.average_position()
            if end_lat is None or end_lon is None:
                self.get_logger().error('Failed to get final GPS data.')
                continue

            # Calculate heading from mid to end position
            heading2 = self.calculate_heading(end_lat, end_lon, mid_lat, mid_lon)

            # Add the new headings to the list of estimates
            self.heading_estimates.append(heading1)
            self.heading_estimates.append(heading2)

            # Calculate the average heading
            avg_heading = sum(self.heading_estimates) / len(self.heading_estimates)
            self.get_logger().info(f'Average heading: {avg_heading}')

            # Check if the average heading has converged (within a small threshold)
            if len(self.heading_estimates) >= 4:  # Ensure we have at least 4 estimates
                recent_estimates = self.heading_estimates[-4:]
                if max(recent_estimates) - min(recent_estimates) < 5:  # Threshold of 5 degrees
                    self.get_logger().info(f'Final converged heading: {avg_heading}')
                    self.publish_heading(avg_heading)
                    break

    def publish_heading(self, heading):
        msg = String()
        msg.data = str(heading)
        self.heading_pub.publish(msg)
        self.get_logger().info(f'Published heading: {heading}')

def main(args=None):
    rclpy.init(args=args)
    gps_subscriber = GpsSubscriber()

    # Perform initialization maneuver
    gps_subscriber.initialization_maneuver()

    rclpy.spin(gps_subscriber)
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

