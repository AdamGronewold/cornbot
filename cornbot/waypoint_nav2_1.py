import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
from rclpy.clock import Clock #used to synchronize everything to the systemtime
import math
import matplotlib.pyplot as plt  # Ensure this import is included
import matplotlib.patches as patches
import numpy as np
from matplotlib.colors import colorConverter

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
            {"lat":  43.70413066512528, "lon":  -72.29571261258029},  # Waypoint 1 (start)
            {"lat": 43.70412235788155, "lon": -72.29570855890388},  # Waypoint 2
            ]  
                
        self.look_ahead_distance = 1.75 #meters
        self.waypoint_threshold = 0.5 #meters
        
        self.typ_robot_speed = -0.25 #nominal speed set for the robot in m/s. Negative is forward, positive is backward.
        self.wheel_base = 0.381  # Example wheel base in meters
        
    #------------------------------------------
    # SUBSCRIBER CALLBACKS
    def update_position_callback(self, msg):
        self.current_lat = msg.point.x
        self.current_lon = msg.point.y
        #self.get_logger().info(f'Current lat,lon: {self.current_lat}, longitude: {self.current_lon}')
        if self.lat0 and self.lon0:
            #robot position from start in meters
            [self.xr, self.yr] = self.latlon_to_xy(self.lat0, self.lon0, self.current_lat, self.current_lon)
            self.get_logger().info(f'x_r: {round(self.xr,2)}, y_r: {round(self.yr,2)}')

    def update_course_callback(self, msg):
        self.current_heading = msg.point.x
        theta_cw = self.current_heading
        self.cur_head_ccw = (90 - theta_cw) % 360
        #self.get_logger().info(f'Last {self.window} headings: {self.last_headings}')
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
                               
        #covert the list of waypoints into an x,y local coordinate frame centered at the origin where +x is east, -x is west, +y is north, -y is south
        self.waypoints_local = []
        for i in range(len(self.waypoints)):
            local_coords = self.latlon_to_xy(self.lat0, self.lon0, self.waypoints[i]["lat"], self.waypoints[i]["lon"])
            waypoint_local = {"x": local_coords[0], "y": local_coords[1]}
            self.waypoints_local.append(waypoint_local)
            self.get_logger().info(f'Waypoint {i} is ~{waypoint_local} meters away (+x=east, +y=north)')   
        self.get_logger().info("\n-------------------------------------------")

    #------------------------------------------------------------
    # GOAL CHECK
    def reached_goal(self):
        dist_2_goal=math.sqrt((abs(self.xr-self.waypoints_local[-1]["x"]))**2 + (abs(self.yr-self.waypoints_local[-1]["y"]))**2)
        if dist_2_goal<=self.waypoint_threshold:
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
        #self.get_logger().info(f"Closest point on path: {self.test_seg_closest_point}, Test segment: {self.test_segment}")
        
        # 6. Determine the active segment
        if self.test_seg_closest_distance <= self.look_ahead_distance:
            # 7. The robot is within one lookahead distance from the closest point on the test segment
            self.active_segment = self.test_segment  # Set the active segment to the test segment
            # Check if the robot is close to the end of the current segment
            dist_to_wp=math.sqrt((abs(self.xr-self.test_segment[1][0]))**2 + (abs(self.yr-self.test_segment[1][1]))**2)
            if dist_to_wp <= self.look_ahead_distance: #if it is...
                # ...move to the next segment if close to the current waypoint

                current_index = next(i for i, waypoint in enumerate(self.waypoints_local) if waypoint["x"] == self.test_segment[1][0])
                if current_index < len(self.waypoints_local) - 1: #only move on to the next segment if a next segment exists
                    next_waypoint = self.waypoints_local[current_index + 1]
                    self.active_segment = (self.test_segment[1], (next_waypoint["x"], next_waypoint["y"]))
                else:
                    self.active_segment=((self.xr, self.yr), self.test_segment[1])         
        else:
            # 8. The robot is farther than one lookahead distance from the closest point on the test segment
            self.active_segment = ((self.xr, self.yr), self.test_seg_closest_point)  # Set the active segment to the robot's position and the closest point on the closest path

        # Log the active segment determined
        #self.get_logger().info(f"Active Segment: {self.active_segment}")

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

        self.get_logger().info(f"Lx: {round(Lx,2)}, Ly: {round(Ly,2)}")
        self.lookahead_point=(Lx,Ly)
    #-----------------------------------------------------------------
    # WHEEL SPEED CALCULATIONS
    def calculate_wheel_speeds(self):
        # Retrieve necessary values from the class instance
        A = (self.xr, self.yr)  # current position
        B = self.lookahead_point  # lookahead point
        theta_h = self.cur_head_ccw  # ranges from [0, 360) from positive x (east)

        theta_h_rad = math.radians(theta_h)

        # Calculate the tangential unit vector from the heading angle (assumed in radians)
        T_x = math.cos(theta_h_rad)
        T_y = math.sin(theta_h_rad)

        # Calculate the vector from the current position to the lookahead point
        dx = B[0] - A[0]
        dy = B[1] - A[1]

        # Calculate the perpendicular distance from the lookahead point to the tangential vector
        cross = np.cross((dx, dy), (T_x, T_y))
        dot_product = np.dot((dx, dy), (T_x, T_y))
        mag_a = np.linalg.norm((dx, dy))
        mag_b = np.linalg.norm((T_x, T_y))
        angle_dif_rad = np.arccos(dot_product / (mag_a * mag_b))
        angle_dif_deg = np.degrees(angle_dif_rad)

        # Calculate the midpoint of the chord AB
        midpoint = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)

        # Calculate the radius of the circle
        perpendicular_distance = self.look_ahead_distance/2
        radius = perpendicular_distance / math.cos(abs(math.radians(90)-angle_dif_rad))

        #if cross > 0 and angle_dif_deg < 90:
            # Circle center to right of chord AB                  
        #elif cross > 0 and angle_dif_deg > 90:
            # Circle center to left of chord AB           
        #elif cross < 0 and angle_dif_deg < 90:
            # Circle center to left of chord AB
        #elif cross < 0 and angle_dif_deg > 90:
            # Circle center to right of chord AB
        # Calculate the perpendicular vector to T_x, T_y
        if cross > 0:
            perp_vector = (T_y, -T_x)
        else:
            perp_vector = (-T_y, T_x)
        self.perp_vector=perp_vector

        center_x = A[0] + radius * perp_vector[0]
        center_y = A[1] + radius * perp_vector[1]

        #print(f"Radius of the circle: {radius}")
        #print(f"Circle center: ({center_x}, {center_y})")
        self.x_c = center_x
        self.y_c = center_y
        self.turn_radius = radius
        
        if radius != 0 and (0<=angle_dif_deg<=90 or 270<=angle_dif_deg<=360): #ie, if it is facing forward use normal pure pursuit
            if cross > 0:
                right = self.typ_robot_speed * (radius + self.wheel_base/2) / radius #turn right
                left = self.typ_robot_speed * (radius - self.wheel_base/2) / radius 
            elif cross < 0:
                right = self.typ_robot_speed * (radius - self.wheel_base/2) / radius #turn left
                left = self.typ_robot_speed * (radius + self.wheel_base/2) / radius
        elif 90<angle_dif_deg<270: #if you are facing the wrong way turn around!!!
            radius=0.5 #turn around by making the radius small
            if cross > 0:
                right = self.typ_robot_speed * (radius + self.wheel_base/2) / radius #turn right
                left = self.typ_robot_speed * (radius - self.wheel_base/2) / radius 
            elif cross < 0:
                right = self.typ_robot_speed * (radius - self.wheel_base/2) / radius #turn left
                left = self.typ_robot_speed * (radius + self.wheel_base/2) / radius
        else:
            right=0
            left=0
            
        return left, right

    #---------------------------------------------------------------
    # PLOTTING STUFF
    
    def plot_navigation(self):
        if plt.fignum_exists(1):
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
                
            #------------------------------------------------------------
                
            #plot current arc to travel on
            if self.x_c and self.y_c:
                fc = colorConverter.to_rgba('yellow', alpha=0.2)
                circle=patches.Circle((self.x_c, self.y_c), self.turn_radius, edgecolor='orange', facecolor=fc, linewidth=2, label='Current Turning Arc')
                self.ax.add_patch(circle)
                self.ax.scatter(self.x_c, self.y_c, color='orange', marker='+', s=100)
                self.ax.text(self.x_c, self.y_c, f"({self.x_c:.2f}, {self.y_c:.2f})", fontsize=9, verticalalignment='bottom')
                self.ax.plot([self.xr, self.x_c], [self.yr, self.y_c], color='orange', linewidth=2)

            # Plot waypoints
            label=None
            for wp in wp_xy:
                if label==None:
                    label="Waypoints"
                    self.ax.scatter(wp["x"], wp["y"], color='lime', marker='o', s=100, label=label)
                    self.ax.text(wp["x"], wp["y"], f"({wp['x']:.2f}, {wp['y']:.2f})", fontsize=9, verticalalignment='bottom')
                else:
                    self.ax.scatter(wp["x"], wp["y"], color='lime', marker='o', s=100)
                    self.ax.text(wp["x"], wp["y"], f"({wp['x']:.2f}, {wp['y']:.2f})", fontsize=9, verticalalignment='bottom')

            # Plot current robot location
            self.ax.scatter(*robot_location, color='red', marker='x', s=200, label='Robot Location')
            self.ax.text(*robot_location, f"({robot_location[0]:.2f}, {robot_location[1]:.2f})", fontsize=9, verticalalignment='bottom')

            # Plot test segment
            self.ax.plot([test_segment[0][0], test_segment[1][0]], [test_segment[0][1], test_segment[1][1]], color='red', linestyle='--', label='Test Path Segment')

            # Plot active segment
            self.ax.plot([active_segment[0][0], active_segment[1][0]], [active_segment[0][1], active_segment[1][1]], color='green', linestyle='-', linewidth=2, label='Active Path Segment')
            self.ax.scatter(*closest_point, color='green', marker='s', s=100, label='Closest Point on Path')
            self.ax.text(*closest_point, f"({closest_point[0]:.2f}, {closest_point[1]:.2f})", fontsize=9, verticalalignment='bottom')

            # Plot lookahead point and vector
            if lookahead_point:
                self.ax.scatter(Lx, Ly, color='magenta', marker='o', s=100, label='Lookahead Point')
                self.ax.text(Lx, Ly, f"({Lx:.2f}, {Ly:.2f})", fontsize=9, verticalalignment='bottom')
                self.ax.plot(lookahead_vector[0], lookahead_vector[1], color='magenta', linestyle='-', linewidth=3)
            else:
                self.ax.text(0,0, "No lookahead point found.", fontsize=9)
            
            # Plot unit vector of based on current heading angle. Heading angle is taken clockwise from positive y (north). Here we convert to counterclockwise from positive x (east)
            theta=math.radians(self.cur_head_ccw)
            u_x = math.cos(theta)
            u_y = math.sin(theta)
            self.ax.quiver(*robot_location, u_x, u_y, angles='xy', scale_units='xy', scale=1, headwidth=2, label='Current Heading', color='purple')
                      
            # Adding labels and legend
            self.ax.set_xlabel('X Coordinate (+x = east)')
            self.ax.set_ylabel('Y Coordinate (+y = north)')
            self.ax.set_title('Robot Navigation Visualization')
            self.ax.legend()
            self.ax.grid(True)
            self.ax.axhline(0, color='black', linewidth=0.5)
            self.ax.axvline(0, color='black', linewidth=0.5)
        
            # Set plot limits centered around the current robot location with 10 meters on each axis
            self.ax.set_xlim(self.xr - 5, self.xr +5)
            self.ax.set_ylim(self.yr - 5, self.yr +5)

            # Draw the updated plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        else:
            self.publish_ref_speed(0.0, 0.0)
            time.sleep(2)
            self.get_logger().info(f'Visualizer closed. Shutting down node...')    
            self.destroy_node()
            rclpy.shutdown()            

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
                try:
                    #"measurement" update
                    rclpy.spin_once(self, timeout_sec=1) #this will update the robots current position (lat, lon) and (x,y) by seeing if messages arrived from the gps node
                
                    #check which waypoint is the waypoint being actively pursued
                    self.find_active_path_segment() #set segment lookahead points at
                    self.find_look_ahead_point() #set the lookahead point in the direction of the active path segment
                
                    # Set and publish the wheel speeds:
                    self.left_speed_ref, self.right_speed_ref=self.calculate_wheel_speeds() #m/s
                    self.plot_navigation() #plot the information in xy
                    self.publish_ref_speed(self.left_speed_ref, self.right_speed_ref)
                                        
                except Exception as e:
                    
                    self.publish_ref_speed(0.0, 0.0)
                    time.sleep(1)
                    self.get_logger().info(f'Exception as {e}. Shutting down node...')    
                    self.destroy_node()
                    rclpy.shutdown()                    
                
            self.get_logger().info("Goal Reached. Stopping robot. Time to rest :) ")
            self.publish_ref_speed(0.0, 0.0)
            time.sleep(5)
            self.destroy_node()
            rclpy.shutdown()
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
