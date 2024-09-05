import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time
import numpy as np

class GpsWaypointCollector(Node):

    def __init__(self):
        super().__init__('gps_waypoint_collector')
        self.subscription = self.create_subscription(
            PointStamped,
            '/gnss/positioning/lat_lon_stamped_topic',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.latitudes = []
        self.longitudes = []

    def listener_callback(self, msg):
        self.latitudes.append(msg.point.x)
        self.longitudes.append(msg.point.y)
        #self.get_logger().info(f'Received latitude: {msg.point.x}, longitude: {msg.point.y}')

    def calculate_average_and_rms(self):
        avg_lat = np.mean(self.latitudes)
        avg_lon = np.mean(self.longitudes)
        lat_errors = np.array(self.latitudes) - avg_lat
        lon_errors = np.array(self.longitudes) - avg_lon
        rms_error = np.sqrt(np.mean(lat_errors**2 + lon_errors**2))
        return avg_lat, avg_lon, rms_error

def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointCollector()
    executor = rclpy.executors.SingleThreadedExecutor()

    start_time = time.time()
    while time.time() - start_time < 60:
        node.get_logger().info(f'Sampling for {round(60-(time.time()-start_time), 2)} more seconds')
        rclpy.spin_once(node, timeout_sec=0.1)

    avg_lat, avg_lon, rms_error = node.calculate_average_and_rms()
    node.get_logger().info(f'Final Average Latitude: {avg_lat}, Final Average Longitude: {avg_lon}')
    node.get_logger().info(f'Final RMS Error: {rms_error} meters')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

