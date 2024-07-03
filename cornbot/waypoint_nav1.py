import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import math

# Waypoints
waypoints = [
    {"lat": 43.70413256564306, "lon": -72.2956549055425},
    {"lat": 43.70407690542583, "lon": -72.2959519327125},
    {"lat": 43.704020785432505, "lon": -72.29625257756}
]

class GpsNavigator(Node):

    def __init__(self):
        super().__init__('gps_navigator')
        self.subscription = self.create_subscription(
            PointStamped,
            'cornbot/gnss/positioning/lat_lon_stamped_topic',
            self.lat_lon_callback,
            1)
        self.course_subscription = self.create_subscription(
            PointStamped,
            'cornbot/gnss/positioning/course_over_ground',
            self.course_callback,
            1)
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.waypoint_index = 0
        self.init_heading_complete = False

    def lat_lon_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y
        self.get_logger().info(f'Received latitude: {self.current_lat}, longitude: {self.current_lon}')
        if self.init_heading_complete:
            self.navigate_to_waypoint()

    def course_callback(self, msg):
        self.current_heading = msg.point.x
        self.get_logger().info(f'Received heading: {self.current_heading} degrees')

    def publish_speed(self, right_ref, left_ref):
        speed_command = f'<{right_ref}, {left_ref}>'
        msg = String()
        msg.data = speed_command
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Published speed: {speed_command}')

    def initialize_heading(self):
        self.get_logger().info("Initializing heading...")
        self.publish_speed(-0.2, -0.2)
        time.sleep(5)
        self.publish_speed(0.0, 0.0)
        self.get_logger().info("Pausing...")
        time.sleep(2)
        self.publish_speed(0.2, 0.2)
        time.sleep(5)
        self.publish_speed(0.0, 0.0)
        self.get_logger().info('Heading initialization completed.')
        self.init_heading_complete = True

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def navigate_to_waypoint(self):
        if self.waypoint_index >= len(waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        waypoint = waypoints[self.waypoint_index]
        distance = self.calculate_distance(self.current_lat, self.current_lon, waypoint["lat"], waypoint["lon"])

        if distance <= 1:
            self.get_logger().info(f"Waypoint {self.waypoint_index + 1} reached.")
            self.waypoint_index += 1
            if self.waypoint_index >= len(waypoints):
                self.get_logger().info("All waypoints reached.")
                self.publish_speed(0.0, 0.0)
                return
            waypoint = waypoints[self.waypoint_index]

        target_bearing = self.calculate_bearing(self.current_lat, self.current_lon, waypoint["lat"], waypoint["lon"])

        if self.current_heading is not None:
            heading_diff = (target_bearing - self.current_heading + 360) % 360
            if heading_diff > 180:
                heading_diff -= 360

            self.get_logger().info(f'Heading difference: {heading_diff} degrees')

            if abs(heading_diff) > 10:
                turn_speed = -0.2 if heading_diff > 0 else 0.2
                self.publish_speed(turn_speed, -turn_speed)
            else:
                self.publish_speed(-0.2, -0.2)
        else:
            self.get_logger().info("Waiting for initial heading...")
            self.publish_speed(-0.2, -0.2)

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GpsNavigator()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gps_navigator)

    gps_navigator.initialize_heading()

    try:
        executor.spin()
    except Exception as r:
        gps_navigator.get_logger().info(f'Destroying node: {r}')
        gps_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

