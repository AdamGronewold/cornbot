import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
from rclpy.clock import Clock
import math

class WaypointNode(Node):

    def __init__(self):
        super().__init__('wps')
        self.subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.lat_lon_callback, 1)
        self.course_subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/course_over_ground', self.course_callback, 1)
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        self.ave_heading_pub = self.create_publisher(PointStamped, 'cornbot/pure_pursuit/move_ave_GNSS_heading', 1)
        
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        
        self.waypoints = [
            {"lat": 43.70407690542583, "lon": -72.2959519327125},  # Waypoint 2
            {"lat": 43.704020785432505, "lon": -72.29625257756}    # Waypoint 3 (goal)
        ]
        self.look_ahead_distance = 2  # meters
        self.goal_threshold = 0.5  # meters
        
        self.window = 5  # moving average window of observations
        self.last_headings = [None] * self.window  # list of last 5 heading estimates
        self.move_ave_heading = None  # average of last 5 heading estimates. In motion, the array is replaced at 4 Hz (fully in 1.25 sec).

    def lat_lon_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y

    def course_callback(self, msg):
        self.current_heading = msg.point.x
        self.last_headings = [self.current_heading] + self.last_headings[:-1]
        self.move_ave_heading = sum(filter(None, self.last_headings)) / len(filter(None, self.last_headings))
        self.publish_ave_heading()

    def publish_ref_speed(self, linear_velocity, angular_velocity):
        speed_msg = String()
        speed_msg.data = f"{linear_velocity},{angular_velocity}"
        self.speed_pub.publish(speed_msg)

    def publish_ave_heading(self):
        heading_msg = PointStamped()
        heading_msg.point.x = self.move_ave_heading
        heading_msg.header.stamp = Clock().now().to_msg()
        self.ave_heading_pub.publish(heading_msg)

    def latlon_to_xy(self, lat, lon, origin_lat, origin_lon):
        R = 6371000  # radius of Earth in meters
        x = (lon - origin_lon) * math.cos(math.radians(origin_lat)) * R
        y = (lat - origin_lat) * R
        return x, y

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def find_lookahead_point(self):
        # Placeholder for finding lookahead point logic
        pass

    def calculate_steering_angle(self, current_position, lookahead_point):
        # Placeholder for calculating steering angle logic
        pass

    def reached_goal(self):
        goal_position = self.waypoints[-1]
        distance_to_goal = self.haversine_distance(self.current_lat, self.current_lon, goal_position["lat"], goal_position["lon"])
        return distance_to_goal <= self.goal_threshold

    def action_loop(self):
        while not self.reached_goal():
            # Placeholder for main loop logic
            pass

    def min_haversine(self):
        min_distance = float('inf')  # Initialize to a large value
        min_index = -1  # To keep track of the index of the waypoint with the minimum distance

        for i, waypoint in enumerate(self.waypoints): 
            hav_dist = self.haversine_distance(self.current_lat, self.current_lon, waypoint["lat"], waypoint["lon"])
            if hav_dist < min_distance:
                min_distance = hav_dist
                min_index = i
        self.get_logger().info(f'Minimum haversine distance: {min_distance} meters to waypoint {min_index}')

def main():
    rclpy.init()
    wps = WaypointNode()

    try:
        wps.action_loop()
    except KeyboardInterrupt:
        wps.publish_ref_speed(0.0, 0.0)
        time.sleep(1)
        wps.get_logger().info("Shutting down node...")
        wps.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        wps.publish_ref_speed(0.0, 0.0)
        time.sleep(1)
        wps.get_logger().info(f'Exception: {e}. Shutting down node...')
        wps.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

