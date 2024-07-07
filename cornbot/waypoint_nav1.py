import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
from rclpy.clock import Clock #used to synchronize everything to the systemtime
import math

class WaypointNode(Node):

    def __init__(self):
        super().__init__('wps')
        self.subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.lat_lon_callback,1)
        self.course_subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/course_over_ground',self.course_callback,1)
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        self.ave_heading_pub = self.create_publisher(PointStamped, 'cornbot/pure_pursuit/move_ave_GNSS_heading', 1)
        
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        
        self.waypoints = [
            {"lat": 43.7040943966805, "lon": -72.29562426452841},  # Waypoint 1 (start)
            {"lat": 43.70407690542583, "lon": -72.2959519327125},  # Waypoint 2
            {"lat": 43.704020785432505, "lon": -72.29625257756}]    # Waypoint 3 (goal)
        
        self.look_ahead_distance = 2 #meters
        self.goal_threshold = 0.5 #meters
        
        self.window = 5 #moving average window of oberservations
        self.last_headings = [None] * self.window #list of last 5 heading estimates
        self.move_ave_heading = None #average of last 5 heading estimates. In motion, the array is replaced at 4 Hz (fully in 1.25 sec).
       
    def lat_lon_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y
        self.get_logger().info(f'Current lat,lon: {self.current_lat}, longitude: {self.current_lon}')
        if self.lat0!=None and self.lon0!=None:
            self.robot_position_xy = self.latlon_to_xy(self.lat0, self.lon0, self.current_lat, self.current_lon)
            self.get_logger().info(f'Current x,y (+e,+n in meters): {wps.robot_position_xy}')
        time.sleep(5)

    def course_callback(self, msg):
        self.current_heading = msg.point.x
        self.last_headings=[self.current_heading] + self.last_headings[:-1] #update stored values
        #self.get_logger().info(f'Last {self.window} headings: {self.last_headings}')
        if all (type(i) is float for i in self.last_headings):
            self.move_ave_heading=sum(self.last_headings)/self.window
            self.get_logger().info(f'Moving average heading: {self.move_ave_heading} degrees')
            self.publish_heading_ave(self.move_ave_heading)
        #self.get_logger().info(f'Received GNSS heading: {self.current_heading} degrees')

    def publish_ref_speed(self, right_ref, left_ref):
        speed_command = f'<{right_ref}, {left_ref}>'
        msg = String()
        msg.data = speed_command
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Published speed: {speed_command}')
        
    def publish_heading_ave(self, ave):
        msg=PointStamped()
        msg.header.stamp=Clock().now().to_msg()
        msg.header.frame_id='/gnss'
        msg.point.x=self.move_ave_heading
        self.ave_heading_pub.publish(msg)
    #-------------------------------------------------------------------------------------

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        #haversine_dtance
        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c      #distance as the crow flies

    def reached_goal(self): #function to check if we are at the final waypoint, within the goal threshold
        self.distance_to_goal = self.haversine_distance(float(self.current_lat),float(self.current_lon),float(self.waypoints[-1]["lat"]),float(self.waypoints[-1]["lon"]))
        return self.distance_to_goal <= self.goal_threshold



    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon1_rad = math.radians(lon1)
        lon2_rad = math.radians(lon2)

        # Calculate differences in radians
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        # Calculate x and y distances
        x = delta_lon * math.cos(lat1_rad) * R
        y = delta_lat * R

        return [x, y]
        
    def waypoints_in_local(self):
        self.waypoints_local = []
        for i in range(len(self.waypoints)):
            local_coords = self.latlon_to_xy(self.lat0, self.lon0, self.waypoints[i]["lat"], self.waypoints[i]["lon"])
            waypoint_local = {"x": local_coords[0], "y": local_coords[1]}
            self.waypoints_local.append(waypoint_local)
            self.get_logger().info(f'Waypoint {i} is ~{waypoint_local} meters away (+x=east, +y=north)')   
    
    def select_current_waypoint(self):
        # Calculate distances to waypoints and find the minimum
        self.min_distance = float('inf')  # Initialize to a large value
        self.min_index = -1  # To keep track of the index of the waypoint with the minimum distance

        for i in range(len(self.waypoints)):  # Loop through the waypoints and find the one that is closest
            hav_dist = self.haversine_distance(
                float(self.current_lat), 
                float(self.current_lon), 
                float(self.waypoints[i]["lat"]), 
                float(self.waypoints[i]["lon"])
            )
            if hav_dist < self.min_distance:
                self.min_distance = hav_dist
                self.min_index = i

        # Check if the robot is within one look-ahead distance of the nearest segment
        def point_line_distance(x0, y0, x1, y1, x2, y2):
            """Calculate the minimum distance from point (x0, y0) to the line segment (x1, y1) to (x2, y2)."""
            px = x2 - x1
            py = y2 - y1
            norm = px * px + py * py
            u = ((x0 - x1) * px + (y0 - y1) * py) / norm
            u = max(min(u, 1), 0)
            x = x1 + u * px
            y = y1 + u * py
            dx = x - x0
            dy = y - y0
            return math.sqrt(dx * dx + dy * dy), x, y
        
        def convert_to_local(lat, lon):
            """Convert latitude and longitude to local x, y coordinates."""
            return self.latlon_to_xy(self.lat0, self.lon0, lat, lon)

        closest_segment = None
        min_segment_distance = float('inf')
        closest_point_on_segment = None

        for i in range(len(self.waypoints) - 1):
            x1, y1 = convert_to_local(self.waypoints[i]["lat"], self.waypoints[i]["lon"])
            x2, y2 = convert_to_local(self.waypoints[i + 1]["lat"], self.waypoints[i + 1]["lon"])
            xr, yr = convert_to_local(self.current_lat, self.current_lon)
            
            dist_to_segment, closest_x, closest_y = point_line_distance(xr, yr, x1, y1, x2, y2)
            if dist_to_segment < min_segment_distance:
                min_segment_distance = dist_to_segment
                closest_segment = (i, i + 1)
                closest_point_on_segment = (closest_x, closest_y)

        if min_segment_distance <= self.look_ahead_distance:
            # Use the closest segment for navigation
            self.active_waypoint = closest_segment[1]
            self.closest_point_on_segment = None
        else:
            # Use the robot's current position and the closest point on the nearest segment
            self.active_waypoint = closest_segment[1]
            self.closest_point_on_segment = closest_point_on_segment

        self.get_logger().info(f'Pursuing waypoint #{self.active_waypoint} at {self.waypoints[self.active_waypoint]}')

    def find_look_ahead_point(self):
        if self.closest_point_on_segment:
            x1, y1 = self.closest_point_on_segment
        else:
            x1 = self.waypoints_local[self.active_waypoint - 1]["x"]
            y1 = self.waypoints_local[self.active_waypoint - 1]["y"]

        x2 = self.waypoints_local[self.active_waypoint]["x"]
        y2 = self.waypoints_local[self.active_waypoint]["y"]
        xr = self.robot_position_xy[0]
        yr = self.robot_position_xy[1]
        Ld = self.look_ahead_distance

        # Calculating the coefficients for the quadratic equation
        a = (x2 - x1) ** 2 + (y2 - y1) ** 2
        b = 2 * ((x1 - xr) * (x2 - x1) + (y1 - yr) * (y2 - y1))
        c = (x1 - xr) ** 2 + (y1 - yr) ** 2 - Ld ** 2

        # Solving the quadratic equation for t
        discriminant = b ** 2 - 4 * a * c

        if discriminant < 0:
            self.get_logger().info("No valid solution for t, the discriminant is negative.")
            return None

        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b + sqrt_discriminant) / (2 * a)
        t2 = (-b - sqrt_discriminant) / (2 * a)

        # Select the valid t in the range [0, 1]
        t = None
        if 0 <= t1 <= 1:
            t = t1
        elif 0 <= t2 <= 1:
            t = t2

        if t is None:
            self.get_logger().info("No valid t in the range [0, 1].")
            return None

        # Calculating the lookahead point coordinates
        Lx = x1 + t * (x2 - x1)
        Ly = y1 + t * (y2 - y1)

        self.get_logger().info(f"Lookahead Point: Lx = {Lx}, Ly = {Ly}")
        return Lx, Ly

     
    def action_init(self):
        #initialize lat, lon
        rclpy.spin_once(self, timeout_sec=1) #this will update the robots current position (lat, lon) by seeing if messages arrived from the gps node
        while self.current_lat == None and self.current_lon == None: #sit stationary until lat and lon start arriving on the topic
            self.get_logger().info("Lat, lon not received from GNSS node. Spinning....")
            rclpy.spin_once(self, timeout_sec=1)
        #once receiving data, initialize start point in lat, lon and in x, y
        self.lat0=self.current_lat
        self.lon0=self.current_lon
        self.localx0=0
        self.localy0=0
        
        #covert the list of waypoints into an x,y local coordinate frame centered at the origin where +x is east, -x is west, +y is north, -y is south
        self.waypoints_in_local() 
        time.sleep(1)
        
        #initialize the heading by getting a dynamic course over ground by moving forward a bit
        self.driveable=False
        if self.driveable==True and self.current_heading == None: #if we don't yet have a reference heading from the GNSS node, initialize by moving forward until we do.
            self.get_logger().info('No current heading. Initializing heading.')
            time.sleep(1)
            #move forward slowly to get an initial heading from the gps/gnss
            while self.current_heading == None:
                self.get_logger().info('No current heading. Moving.')
                self.publish_ref_speed(-0.1, -0.1)
                rclpy.spin_once(self, timeout_sec=1)
            while sum(x is None for x in self.last_headings)!=0:
                self.get_logger().info('No current average heading. Moving.')
                self.publish_ref_speed(-0.1, -0.1)
                rclpy.spin_once(self, timeout_sec=1)
            self.publish_ref_speed(0.0, 0.0)
#-----------------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    wps = WaypointNode()
    time.sleep(0.5)
    
    def pure_pursuit():
        while rclpy.ok():
        
            #action initialization
            wps.action_init()
            
            # Main loop of the Pure Pursuit algorithm
            while not wps.reached_goal():
               
                #"measurement" update
                rclpy.spin_once(wps, timeout_sec=1) #this will update the robots current position (lat, lon) and (x,y) by seeing if messages arrived from the gps node
                
                #check which waypoint is the waypoint being actively pursued
                wps.select_current_waypoint()
                               
                # Parameterize the current active segment of the path to follow
                wps.find_look_ahead_point()
                                
                #wps.vect_to_waypoint_xy = wps.latlon_to_xy(wps.current_lat, wps.current_lon, wps.waypoints[wps.active_waypoint]["lat"], wps.waypoints[wps.active_waypoint]["lon"])
                #wps.get_logger().info(f'Vector to active waypoint (m): {wps.vect_to_waypoint_xy}')
    
                #steering_angle = calculate_steering_angle(current_position_xy, lookahead_point)

                #send_steering_command(steering_angle)
                  
    try:
        pure_pursuit()
    except Exception as e:
        wps.publish_ref_speed(0.0, 0.0)
        time.sleep(1)
        wps.get_logger().info(f'Exception as {e}. Shutting down node...')    
        wps.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
