import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
from rclpy.clock import Clock #used to synchronize everything to the systemtime
import math
import matplotlib.pyplot as plt  # Ensure this import is included

class WaypointNode(Node):
    #------------------------------
    # Node and variable initialization
    def __init__(self):
        super().__init__('wps')
        self.subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.update_position_callback,1)
        self.course_subscription = self.create_subscription(PointStamped, 'cornbot/gnss/positioning/course_over_ground',self.update_course_callback,1)
        self.speed_pub = self.create_publisher(String, 'cornbot/speed_ref_topic', 1)
        self.ave_heading_pub = self.create_publisher(PointStamped, 'cornbot/pure_pursuit/move_ave_GNSS_heading', 1)
        
        #robot start lat, lon
        self.lat0 = None
        self.lon0 = None
        
        #current robot lat, lon
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
         
        #current robot x, y (east, north) in meters       
        self.xr= None
        self.yr = None
        
        self.waypoints = [
            {"lat": 43.70411893367632, "lon": -72.29562979300668},  # Waypoint 1 (start)
            {"lat": 43.70407690542583, "lon": -72.2959519327125},  # Waypoint 2
            {"lat": 43.704020785432505, "lon": -72.29625257756}]    # Waypoint 3 (goal)
                
        self.look_ahead_distance = 2 #meters
        self.goal_threshold = 0.5 #meters
        
        self.window = 5 #moving average window of oberservations
        self.last_headings = [None] * self.window #list of last 5 heading estimates
        self.move_ave_heading = None #average of last 5 heading estimates. In motion, the array is replaced at 4 Hz (fully in 1.25 sec).

        self.typ_robot_speed = -0.1 #nominal speed set for the robot in m/s. Negative is forward, positive is backward.
        self.wheel_base = 0.381  # Example wheel base in meters
        
    #------------------------------------------
    # SUBSCRIBER CALLBACKS
    def update_position_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y
        self.get_logger().info(f'Current lat,lon: {self.current_lat}, longitude: {self.current_lon}')
        if self.lat0 and self.lon0:
            #robot position from start in meters
            [self.xr, self.yr] = self.latlon_to_xy(self.lat0, self.lon0, self.current_lat, self.current_lon)
            self.get_logger().info(f'Current x,y (+e,+n in meters): {self.xr}, {self.yr}')

    def update_course_callback(self, msg):
        self.current_heading = msg.point.x
        self.last_headings=[self.current_heading] + self.last_headings[:-1] #update stored values
        #self.get_logger().info(f'Last {self.window} headings: {self.last_headings}')
        if all (type(i) is float for i in self.last_headings):
            self.move_ave_heading=sum(self.last_headings)/self.window
            self.get_logger().info(f'Moving average heading: {self.move_ave_heading} degrees')
            self.publish_heading_ave(self.move_ave_heading)
        #self.get_logger().info(f'Received GNSS heading: {self.current_heading} degrees')
    #---------------------------------------------------------------------
    # FUNCTIONS TO PUBLISH
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
    
    #-------------------------------------------------------------------------
    # HELPER FUNCTIONS    
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
                
    #------------------------------------------------------------------------------
    # STARTUP WIGGLES
    
    def action_init(self):
    
        #initialize lat, lon
        while self.current_lat == None and self.current_lon == None: #sit stationary until lat and lon start arriving on the topic
            self.get_logger().info("Lat, lon not received from GNSS node. Spinning....")
            rclpy.spin_once(self, timeout_sec=1)
        #once receiving data, initialize start point in lat, lon and in x, y
        self.lat0=self.current_lat
        self.lon0=self.current_lon
        
        #initialize the heading by getting a dynamic course over ground by moving forward a bit
        
        if self.current_heading == None: #if we don't yet have a reference heading from the GNSS node, initialize by moving forward until we do.
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
            
                    
        #covert the list of waypoints into an x,y local coordinate frame centered at the origin where +x is east, -x is west, +y is north, -y is south
        self.waypoints_local = []
        for i in range(len(self.waypoints)):
            local_coords = self.latlon_to_xy(self.lat0, self.lon0, self.waypoints[i]["lat"], self.waypoints[i]["lon"])
            waypoint_local = {"x": local_coords[0], "y": local_coords[1]}
            self.waypoints_local.append(waypoint_local)
            self.get_logger().info(f'Waypoint {i} is ~{waypoint_local} meters away (+x=east, +y=north)')   
        self.get_logger().info("\n-------------------------------------------")
        time.sleep(5)
    #------------------------------------------------------------
    # GOAL CHECK
    def reached_goal(self):
        dist_2_goal=math.sqrt((abs(self.xr-self.waypoints_local[-1]["x"]))**2 + (abs(self.yr-self.waypoints_local[-1]["y"]))**2)
        if dist_2_goal<=self.goal_threshold:
             return True
        else:
            return False
        
        
    #------------------------------------------------------------
    # CALCULATE LOOKAHEAD POINT STUFF

    
    def find_active_path_segment(self):
        # 1. Initialize variables to find the segment with the closest point
        self.test_seg_closest_distance = float('inf')  # Set initial closest distance to infinity
        self.test_seg_closest_point = None  # Initialize variable to store the closest point
        self.test_segment = None  # Initialize variable to store the closest segment

        # 2. Iterate through each segment of the path
        for i in range(len(self.waypoints_local) - 1):  # Loop through waypoints, stopping before the last one
            wp1 = self.waypoints_local[i]  # Current waypoint
            wp2 = self.waypoints_local[i + 1]  # Next waypoint

            x1, y1 = wp1["x"], wp1["y"]  # Coordinates of the current waypoint
            x2, y2 = wp2["x"], wp2["y"]  # Coordinates of the next waypoint

            # 3. Find the point on the segment closest to the robot
            px, py = self.xr, self.yr  # Robot's current position
            t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)  # Projection factor
            t = max(0, min(1, t))  # Clamp t to the range [0, 1]
            segment_closest_x = x1 + t * (x2 - x1)  # X coordinate of the closest point on the segment
            segment_closest_y = y1 + t * (y2 - y1)  # Y coordinate of the closest point on the segment

            # 4. Calculate the distance from the robot to this closest point        
            distance = math.sqrt((px - segment_closest_x) ** 2 + (py - segment_closest_y) ** 2)  # Euclidean distance

            # 5. Update if this segment has the closest point found so far
            if distance < self.test_seg_closest_distance:  # Check if this is the closest point so far
                self.test_seg_closest_distance = distance  # Update the closest distance
                self.test_seg_closest_point = (segment_closest_x, segment_closest_y)  # Update the closest point
                self.test_segment = ((x1, y1), (x2, y2))  # Update the closest segment

        # Log the closest point and segment found
        self.get_logger().info(f"Closest point on path: {self.test_seg_closest_point}, Test segment: {self.test_segment}")
        

        # 6. Determine the active segment
        if self.test_seg_closest_distance <= self.look_ahead_distance:
            # 7. The robot is within one lookahead distance from the closest point on the test segment
            self.active_segment = self.test_segment  # Set the active segment to the test segment
        else:
            # 8. The robot is farther than one lookahead distance from the closest point on the test segment
            self.active_segment = ((self.xr, self.yr), self.test_seg_closest_point)  # Set the active segment to the robot's position and the closest point on the closest path

        # Log the active segment determined
        self.get_logger().info(f"Active Segment: {self.active_segment}")
        
    def find_look_ahead_point(self):
        x1, y1 = self.active_segment[0]  # Start of active segment
        x2, y2 = self.active_segment[1]  # End of active segment

        xr = self.xr  # Current robot x position
        yr = self.yr  # Current robot y position
        Ld = self.look_ahead_distance  # Lookahead distance

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
        self.lookahead_point=(Lx,Ly)
    #-----------------------------------------------------------------
    # WHEEL SPEED CALCULATIONS
    def calculate_wheel_speeds(self):
        """
        Calculate left and right wheel speeds to navigate the robot to the lookahead point using inverse kinematics and geometry.

        Returns:
        - left_speed: float, speed of the left wheel in m/s
        - right_speed: float, speed of the right wheel in m/s
        """

        # Retrieve necessary values from the class instance
        current_position = (self.xr, self.yr)
        lookahead_point = self.lookahead_point
        current_heading = self.current_heading
        robot_speed = self.typ_robot_speed
        wheel_base = self.wheel_base  # Ensure this attribute is defined in your class

        # Step 1: Calculate the angle to the lookahead point
        dx = lookahead_point[0] - current_position[0]
        dy = lookahead_point[1] - current_position[1]
        angle_to_lookahead = math.atan2(dy, dx)

        # Step 2: Calculate the heading error (difference between current heading and angle to lookahead)
        heading_error = angle_to_lookahead - current_heading

        # Step 3: Normalize the heading error to the range [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Step 4: Calculate the radius of the circle (R) the robot is turning on
        R = abs(dx**2 + dy**2) / (2 * abs(dx * math.sin(current_heading) - dy * math.cos(current_heading)))

        # Step 5: Calculate the linear velocities for the left and right wheels
        # The left and right wheel velocities can be found using the relationship:
        # v_left = v_center * (R - L/2) / R
        # v_right = v_center * (R + L/2) / R

        if heading_error == 0:
            # If heading error is zero, the robot is moving straight
            left_speed = robot_speed
            right_speed = robot_speed
        else:
            # Calculate the wheel speeds for turning
            left_speed = robot_speed * (R - wheel_base / 2) / R
            right_speed = robot_speed * (R + wheel_base / 2) / R

        return left_speed, right_speed
        
    #---------------------------------------------------------------
    # PLOTTING STUFF
    
    def plot_navigation(self):
        # Clear the previous plot
        self.ax.clear()

        # Waypoint locations in x, y coordinates
        wp_xy = self.waypoints_local

        # Current robot location
        robot_location = (self.xr, self.yr)

        # Closest point on path (calculated previously in find_active_path_segment)
        closest_point = self.test_seg_closest_point  # Update this as per your calculation

        # Active segment (calculated previously in find_active_path_segment)
        active_segment = self.active_segment

        # Test segment (from find_active_path_segment)
        test_segment = self.test_segment  # Update this as per your calculation

        # Lookahead point
        lookahead_point = self.lookahead_point
        if lookahead_point:
            Lx, Ly = lookahead_point
            # Lookahead vector
            lookahead_vector = ([self.xr, Lx], [self.yr, Ly])

        # Plot waypoints
        for wp in wp_xy:
            labels = [text.get_text() for text in self.ax.get_legend().get_texts()] if self.ax.get_legend() else []
            self.ax.scatter(wp["x"], wp["y"], color='blue', marker='o', label='Waypoint' if 'Waypoint' not in labels else "")
            self.ax.text(wp["x"], wp["y"], f"({wp['x']:.2f}, {wp['y']:.2f})", fontsize=9, verticalalignment='bottom')

        # Plot current robot location
        self.ax.scatter(*robot_location, color='red', marker='x', label='Robot Location')
        self.ax.text(*robot_location, f"({robot_location[0]:.2f}, {robot_location[1]:.2f})", fontsize=9, verticalalignment='bottom')

        # Plot test segment
        self.ax.plot([test_segment[0][0], test_segment[1][0]], [test_segment[0][1], test_segment[1][1]], color='red', linestyle='--', label='Test Segment')

        # Plot active segment
        self.ax.plot([active_segment[0][0], active_segment[1][0]], [active_segment[0][1], active_segment[1][1]], color='green', linestyle='-', linewidth=2, label='Active Segment')
        self.ax.scatter(*closest_point, color='green', marker='s', label='Closest Point on Path')
        self.ax.text(*closest_point, f"({closest_point[0]:.2f}, {closest_point[1]:.2f})", fontsize=9, verticalalignment='bottom')

        # Plot lookahead point and vector
        if lookahead_point:
            self.ax.scatter(Lx, Ly, color='magenta', marker='o', label='Lookahead Point')
            self.ax.text(Lx, Ly, f"({Lx:.2f}, {Ly:.2f})", fontsize=9, verticalalignment='bottom')
            self.ax.plot(lookahead_vector[0], lookahead_vector[1], color='magenta', linestyle='-', linewidth=1, label='Lookahead Vector')
        else:
            self.ax.text(0,0, "No lookahead point found.", fontsize=9)
        # Adding labels and legend
        self.ax.set_xlabel('X Coordinate (+x = east)')
        self.ax.set_ylabel('Y Coordinate (+y = north)')
        self.ax.set_title('Robot Navigation Visualization')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.axhline(0, color='black', linewidth=0.5)
        self.ax.axvline(0, color='black', linewidth=0.5)

        # Draw the updated plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def initialize_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.ion()
        self.fig.show()
        self.fig.canvas.draw()
        
    #------------------------------------------------------------
    def pure_pursuit(self):
        while rclpy.ok():
        
            #action initialization
            self.action_init()
            self.initialize_plot()
            # Main loop of the Pure Pursuit algorithm
            while not self.reached_goal():
               
                #"measurement" update
                rclpy.spin_once(self, timeout_sec=1) #this will update the robots current position (lat, lon) and (x,y) by seeing if messages arrived from the gps node
                
                #check which waypoint is the waypoint being actively pursued
                self.find_active_path_segment() #set segment lookahead points at
                self.find_look_ahead_point() #set the lookahead point in the direction of the active path segment
                self.plot_navigation() #plot the information in xy
    

                # Set and publish the wheel speeds:
                left_speed, right_speed = self.calculate_wheel_speeds()
                #self.get_logger().info(f"Set Left Speed: {left_speed} m/s, Set Right Speed: {right_speed} m/s")
                try:
                    self.publish_ref_speed(left_speed, right_speed)
                except Exception as e:
                    self.publish_ref_speed(0.0, 0.0)
                    time.sleep(1)
                    self.get_logger().info(f'Exception as {e}. Shutting down node...')    
                    self.destroy_node()
                    rclpy.shutdown()                    
                
            self.get_logger().info("Goal Reached. Stopping robot. Time to rest :) ")
            self.publish_ref_speed(0.0, 0.0)
    #----------------------------------------------------------------------------
  
                     
def main(args=None):
    rclpy.init(args=args)
    wps = WaypointNode()
    time.sleep(0.5)
    try:
        wps.pure_pursuit()
    except Exception as e:
        wps.publish_ref_speed(0.0, 0.0)
        time.sleep(1)
        wps.get_logger().info(f'Exception as {e}. Shutting down node...')    
        wps.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
