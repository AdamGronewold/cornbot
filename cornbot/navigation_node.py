import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from geometry_msgs.msg import Point  # Use the Point service from geometry_msgs
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
import math
from std_srvs.srv import Trigger
import json
import time
import numpy as np
from visualization_msgs.msg import Marker

class NavNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # Initial position and heading
        self.initial_lat = None
        self.initial_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_x = None
        self.current_y = None
        self.current_heading = None
        self.odom_plant_map=[]
        self.planned_path=[]
        self.track = 0.381; #Track width of the robot wheel to wheel, in meters
        self.fix_string = 'Fix Quality Unknown'
        self.goal_points_latlon = [
            {"lat": 43.704063422626014, "lon": -72.29563309761032, "angle": 180.001},
            {"lat": 43.70395999479833, "lon": -72.29585203974999, "angle": 180.001}
        ]
        self.goal_points_xy=[] #these will be filled after we obtain our start position
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers for GNSS data
        self.create_subscription(PointStamped, 'gnss/positioning/lat_lon_stamped_topic', self.update_position_callback, 1)
        self.create_subscription(PointStamped, 'gnss/positioning/course_over_ground', self.update_heading_callback, 1)
        self.create_subscription(String, 'gnss/status/fix_quality_topic', self.fix_quality_callback, 5)
        
        #Pubs
        self.speed_pub = self.create_publisher(String, 'speed_ref_topic', 1)
        self.pose_pub = self.create_publisher(PoseStamped, 'gnss/tf_pose', 10)
        self.startup_pose_pub = self.create_publisher(PointStamped, 'gnss/startup_pose', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 100)
        self.pursuit_marker_pub = self.create_publisher(Marker, '/pursuit_marker', 50)  # Added for pursuit markers
        self.path_marker_pub = self.create_publisher(Marker, '/path_marker', 50)  # Added for path markers

        
        #Broadcast so we see changes in RViz
        self.timer = self.create_timer(0.008, self.broadcast_transform)
        
        #client side for us to request the current map from the feeler localization node
        self.map_service_client = self.create_client(Trigger, '/odom_plant_map')
        while not self.map_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Plant map not available. Waiting...')

        #self.create_timer(0.05, self.request_map_update)     
        
        self.start_up_timer=self.create_timer(0.25, self.start_up_loop)  

    def fix_quality_callback(self, msg):
        self.fix_string = msg.data
        if self.fix_string in ('Fix not valid', 'Type 3, Not Applicable', 'GPS Fix','INU Dead reckoning', 'Fix Quality Unknown'):
            self.publish_ref_speed(0.0, 0.0)
            self.get_logger().info(f'Stopping robot with fix type: {self.fix_string}')
            self.timer.cancel()
            self.start_up_timer.cancel()
        elif self.fix_string in ('RTK Fixed, xFill','RTK Float, OmniSTAT XP/HP, Location RTK, RTX', 'Differential GPS by SBAS'):
            if self.timer.is_canceled():
                self.timer = self.create_timer(0.001, self.broadcast_transform)

    def update_position_callback(self, msg):
                
        self.current_lat=msg.point.x
        self.current_lon=msg.point.y
        
        if self.initial_lat is None and self.initial_lon is None:
            self.initial_lat = msg.point.x
            self.initial_lon = msg.point.y

            # Publish the initial lat/lon values
            startup_pose_msg = PointStamped()
            startup_pose_msg.header.stamp = self.get_clock().now().to_msg()
            startup_pose_msg.header.frame_id = 'gnss'
            startup_pose_msg.point.x = self.initial_lat
            startup_pose_msg.point.y = self.initial_lon
            self.startup_pose_pub.publish(startup_pose_msg)
           
        if self.initial_lat is not None and self.initial_lon is not None:    
            # Publish the initial lat/lon values
            startup_pose_msg = PointStamped()
            startup_pose_msg.header.stamp = self.get_clock().now().to_msg()
            startup_pose_msg.header.frame_id = 'gnss'
            startup_pose_msg.point.x = self.initial_lat
            startup_pose_msg.point.y = self.initial_lon
            self.startup_pose_pub.publish(startup_pose_msg)

            #update the current robot position
            self.current_x, self.current_y = self.latlon_to_xy(self.initial_lat, self.initial_lon, msg.point.x, msg.point.y)

    def update_heading_callback(self, msg):
        self.current_heading = (90 - msg.point.x) % 360
        #self.get_logger().info(f'Heading callback: {self.current_heading}')

    def broadcast_transform(self):
        if self.current_x is not None and self.current_y is not None:
            if self.current_heading is None:
                return

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.translation.z = 0.0762

            q = self.euler_to_quaternion(0, 0, math.radians(self.current_heading))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = self.current_x
            pose_msg.pose.position.y = self.current_y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = t.transform.rotation.x
            pose_msg.pose.orientation.y = t.transform.rotation.y
            pose_msg.pose.orientation.z = t.transform.rotation.z
            pose_msg.pose.orientation.w = t.transform.rotation.w

            self.pose_pub.publish(pose_msg)

            startup_pose_msg = PointStamped()
            startup_pose_msg.header.stamp = self.get_clock().now().to_msg()
            startup_pose_msg.point.x = self.initial_lat
            startup_pose_msg.point.y = self.initial_lon
            self.startup_pose_pub.publish(startup_pose_msg)
    
    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon1_rad = math.radians(lon1)
        lon2_rad = math.radians(lon2)

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        x = delta_lon * math.cos(lat1_rad) * R
        y = delta_lat * R

        return x, y

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]
        
    def request_map_update(self):

        # Create a request (no fields to set for Trigger)
        request = Trigger.Request()

        # Call the service and wait for the response
        map_call=self.map_service_client.call_async(request)
        # Attach a callback to process the result when it's ready
        map_call.add_done_callback(self.handle_map_response)

    def handle_map_response(self, future):
        try:
            response = future.result()
            #self.get_logger().info('Service call completed.')
            if response.success:
                self.odom_plant_map = json.loads(response.message)
                self.get_logger().info(f'Current received plant map: {self.odom_plant_map}')
        except Exception as e:
            self.get_logger().error(f'Map service call failed: {e}')

    def publish_ref_speed(self, right_ref, left_ref):  # ✓
        speed_command = f'<{right_ref}, {left_ref}>'
        msg = String()
        msg.data = speed_command
        self.speed_pub.publish(msg)
        # self.get_logger().info(f'Published speed: {speed_command}')        

    def shutdown_robot(self):
        self.publish_ref_speed(0.0, 0.0)
        time.sleep(1)
        self.destroy_node()
        time.sleep(1)
        rclpy.shutdown()
        time.sleep(1)
        
    def cleanup_goal_points(self):
        # Initialize variables
        p_min = None  # Closest point on the straight path between two adjacent goal points
        p_min_dist = float('inf')  # Distance to closest point to be found
        p_seg = None  # The segment the closest point is on to be found

        # Loop through the goals to find the closest segment
        for j in range(len(self.goal_points_xy) - 1):
            goal1 = self.goal_points_xy[j, :]
            goal2 = self.goal_points_xy[j + 1, :]
            
            # Projection factor indicating how far along the segment the closest point is
            segment_vector = goal2[:2] - goal1[:2]
            M = max(0, min(1, np.dot([self.current_x, self.current_y] - goal1[:2], segment_vector) / np.dot(segment_vector, segment_vector)))

            # X and Y coordinates of closest point on segment
            seg_min = goal1[:2] + M * segment_vector

            # Distance from robot to closest point of segment
            p_dist = np.linalg.norm([self.current_x, self.current_y] - seg_min)

            if p_dist < p_min_dist:  # If this is the closest point so far...
                p_min_dist = p_dist  # Update the closest distance,
                p_min = seg_min  # The closest point
                p_seg = (goal1, goal2)  # And the closest segment

        # Check if p_min is the beginning or end of the closest segment
        if np.allclose(p_min, p_seg[0][:2]):
            # Find the index where the next segment starts
            indices = np.where(np.all(self.goal_points_xy[:, :2] == p_seg[1][:2], axis=1))[0]
            if indices.size > 0:
                index = indices[0]
            else:
                index = len(self.goal_points_xy)  # Default to the end of the array if no match is found
            
            # Stack the goal points
            self.goal_points_xy = np.vstack([p_seg[0], self.goal_points_xy[index:]])
            return

        elif np.allclose(p_min, p_seg[1][:2]):
            # Find the index where the segment ends
            indices = np.where(np.all(self.goal_points_xy[:, :2] == p_seg[1][:2], axis=1))[0]
            if indices.size > 0:
                index = indices[0]
            else:
                index = len(self.goal_points_xy)  # Default to the end of the array if no match is found
            
            # Slice the goal points array from the found index
            self.goal_points_xy = self.goal_points_xy[index:]
            return

        else:
            # Interpolate the heading and create a new goal point
            interpolated_heading = p_seg[0][2] + M * (p_seg[1][2] - p_seg[0][2])
            new_goal_point = np.array([*p_min, interpolated_heading])
            
            # Find the index where the segment ends
            indices = np.where(np.all(self.goal_points_xy[:, :2] == p_seg[1][:2], axis=1))[0]
            if indices.size > 0:
                index = indices[0]
            else:
                index = len(self.goal_points_xy)  # Default to the end of the array if no match is found
            
            # Stack the new goal point with the remaining goal points
            self.goal_points_xy = np.vstack([new_goal_point, self.goal_points_xy[index:]])

    def init_nav(self):
        #Ensure lat lon is coming in
        if self.current_lat is None or self.current_lon is None or self.initial_lat is None or self.initial_lon is None:  # sit stationary until lat and lon start arriving on the topic
            self.get_logger().info("Lat, lon not received from GNSS node. Waiting....")
            return
                               
        #self.get_logger().info(f'{self.current_heading}')
        if self.current_heading is None:  # If we don't yet have a reference heading from the GNSS node, initialize by moving forward until we do.
            self.get_logger().info('Initializing heading by inching forward.')
            self.publish_ref_speed(-0.1, -0.1)
            return
           
            
        #convert our lat lon goals to x y goals w.r.t. start location lat lon
        for i in range(len(self.goal_points_latlon)):
            local_coords = self.latlon_to_xy(self.initial_lat, self.initial_lon, self.goal_points_latlon[i]["lat"], self.goal_points_latlon[i]["lon"])
            goal_point = [local_coords[0], local_coords[1], self.goal_points_latlon[i]["angle"]]
            self.goal_points_xy.append(goal_point)
            #self.get_logger().info(f'Goal point {i} is ~{goal_point} meters away (+x=east, +y=north, yaw=RHR from x)')
        
        self.publish_ref_speed(0.0, 0.0)
        self.goal_points_xy = np.array(self.goal_points_xy)
        self.cleanup_goal_points() # Convert the goal list into a useable list of goals
        self.get_logger().info(f'Processed xy goal list: {self.goal_points_xy})')
        self.display_goal_markers()
        time.sleep(3)
        
        self.current_goal = self.goal_points_xy[0, :]  # start with the first goal
        self.current_goal_index = 0  # Assuming this starts at the first goal index
        self.planning_distance = 3  # only plan 3 meters ahead of the robot based on the goal_points
        self.obstactle_decay_buffer = -20  # Exponential padding away from plants in A*
        self.grid_size = 0.05  # A* grid size for search
        self.request_map_update()  # Initialize the global map to store plant locations, sets self.odom_plant_map
        self.local_map_mask = []  # map used in A*
        
        # Pure Pursuit Values
        self.Ld = 0.50  # Lookahead distance in meters
        self.goal_thresh = 0.5  # Goal threshold in meters
        self.v_ref = -0.1  # Nominal speed set for the robot in m/s, negative is forward (sorry)
        self.buffer_size = 150  # only keep 150 timesteps at most
        self.plan_path()
        
    def display_goal_markers(self):
        for i in range(len(self.goal_points_xy)):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_points"
            marker.id = i  # Use the provided marker_id for consistent identification
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.goal_points_xy[i][0]
            marker.pose.position.y = self.goal_points_xy[i][0]
            marker.pose.position.z = 10.0  # Center of the cylinder (half of the height)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.50  # Diameter of the cylinder
            marker.scale.y = 0.50  # Diameter of the cylinder
            marker.scale.z = 20.0  # Height of the cylinder
            marker.color.a = 0.5  # Alpha (opacity)
            marker.color.r = 1.0
            marker.color.g = 0.0  # Green color
            marker.color.b = 0.0

            self.goal_marker_pub.publish(marker)
            
    def start_up_loop(self):
        try:
            # Initialize until we fill these fields, then
            if self.current_lat is None or self.current_lon is None or self.initial_lat is None or self.current_heading is None or self.goal_points_xy == []:
                self.init_nav()
                
            else:   # we will cancel the startup timer, stop the robot and finally start the main loop timer.
                self.publish_ref_speed(0.0, 0.0)
                self.get_logger().info(f'Initialized lat, lon: {(self.initial_lat, self.initial_lon)}')
                self.get_logger().info(f'Initialized heading: {self.current_heading} deg.')
                self.get_logger().info('Finalized startup.')
                self.start_up_timer.cancel()
                
                self.request_map_update()
                self.plan_path()      
                       
                self.main_loop_timer = self.create_timer(0.008, self.main_loop)
                self.path_plan_timer = self.create_timer(0.06, self.plan_path)
                self.map_request_timer=self.create_timer(0.008, self.request_map_update)
                
        except Exception as e:
            self.get_logger().info(f'Exception during GNSS startup execution as {e}. Shutting down node...')
            self.shutdown_robot()
    
    def main_loop(self):
        
        self.odom_plant_map=self.odom_plant_map
        self.planned_path = self.planned_path  # This line is symbolic, basically it says just use the most up-to-date map, but the map is being requested under its own timer at a slower rate.
        
        self.pure_pursuit()

        # Check if the robot has reached the final goal
        if np.sqrt((self.goal_points_xy[-1, 0] - self.current_x)**2 + (self.goal_points_xy[-1, 1] - self.current_y)**2) <= self.goal_thresh:
            self.get_logger().info('Made it to goal')
            self.shutdown_robot()

        # Check if the robot has reached the current goal
        if np.linalg.norm([self.current_x, self.current_y] - self.current_goal[:2]) < self.goal_thresh:
            # Move to the next goal if available
            if self.current_goal_index < self.goal_points_xy.shape[0] - 1:
                self.current_goal_index += 1
                self.current_goal = self.goal_points_xy[self.current_goal_index, :]

    def pure_pursuit(self):
        
        self.find_Ld_point()  # Find lookahead point
        self.calculate_circle_and_speeds()  # Calculate the circle and wheel speeds
        # Publish new reference speeds
        self.draw_pursuit()
        self.publish_ref_speed(self.left_speed_ref, self.right_speed_ref)

    def find_Ld_point(self):
        try:
            path_points = self.planned_path[:, :2]
            current_index = 0
            while current_index < path_points.shape[0] - 1:
                x1 = path_points[current_index, 0]
                y1 = path_points[current_index, 1]
                x2 = path_points[current_index + 1, 0]
                y2 = path_points[current_index + 1, 1]

                a = (x2 - x1)**2 + (y2 - y1)**2
                b = 2 * ((x1 - self.current_x) * (x2 - x1) + (y1 - self.current_y) * (y2 - y1))
                c = (x1 - self.current_x)**2 + (y1 - self.current_y)**2 - self.Ld**2

                discriminant = b**2 - 4 * a * c
                
                if discriminant >= 0:
                    sqrt_discriminant = np.sqrt(discriminant)
                    t1 = (-b + sqrt_discriminant) / (2 * a)
                    t2 = (-b - sqrt_discriminant) / (2 * a)

                    t = None
                    if 0 <= t1 <= 1:
                        t = t1
                    elif 0 <= t2 <= 1:
                        t = t2

                    if t is not None:
                        self.Lx = x1 + t * (x2 - x1)
                        self.Ly = y1 + t * (y2 - y1)

                        if np.dot([self.Lx - self.current_x, self.Ly - self.current_y], 
                                  [np.cos(np.radians(self.current_heading)), np.sin(np.radians(self.current_heading))]) > 0:
                            return

                current_index += 1

            # If no valid Ld point found, set it to the last point in the path
            self.Lx = path_points[-1, 0]
            self.Ly = path_points[-1, 1]
        except:
            self.get_logger().info('Failed to find lookahead.')

    def calculate_circle_and_speeds(self):
        try:
            theta_h = np.radians(self.current_heading)
            T_x = np.cos(theta_h)
            T_y = np.sin(theta_h)

            dx = self.Lx - self.current_x
            dy = self.Ly - self.current_y

            cross = dx * T_y - dy * T_x
            dot_product = dx * T_x + dy * T_y
            mag_a = np.sqrt(dx**2 + dy**2)
            mag_b = np.sqrt(T_x**2 + T_y**2)
            angle_dif_rad = np.arccos(dot_product / (mag_a * mag_b))
            angle_dif_deg = np.degrees(angle_dif_rad)

            perpendicular_distance = self.Ld / 2
            self.turn_rad = perpendicular_distance / np.cos(np.abs(np.radians(90) - angle_dif_rad))

            if cross > 0:
                perp_vector = [T_y, -T_x]
            else:
                perp_vector = [-T_y, T_x]

            self.turn_center_x = self.current_x + self.turn_rad * perp_vector[0]
            self.turn_center_y = self.current_y + self.turn_rad * perp_vector[1]

            if self.turn_rad != 0 and (0 <= angle_dif_deg <= 30 or 330 <= angle_dif_deg <= 360):
                if cross > 0:
                    self.right_speed_ref = self.v_ref * (self.turn_rad + self.track / 2) / self.turn_rad
                    self.left_speed_ref = self.v_ref * (self.turn_rad - self.track / 2) / self.turn_rad
                else:
                    self.right_speed_ref = self.v_ref * (self.turn_rad - self.track / 2) / self.turn_rad
                    self.left_speed_ref = self.v_ref * (self.turn_rad + self.track / 2) / self.turn_rad
            elif 30 < angle_dif_deg < 330:
                self.turn_rad = 0.2
                self.turn_center_x = self.current_x + self.turn_rad * perp_vector[0]
                self.turn_center_y = self.current_y + self.turn_rad * perp_vector[1]
                if cross > 0:
                    self.right_speed_ref = self.v_ref * (self.turn_rad + self.track / 2) / self.turn_rad
                    self.left_speed_ref = self.v_ref * (self.turn_rad - self.track / 2) / self.turn_rad
                else:
                    self.right_speed_ref = self.v_ref * (self.turn_rad - self.track / 2) / self.turn_rad
                    self.left_speed_ref = self.v_ref * (self.turn_rad + self.track / 2) / self.turn_rad
            else:
                self.right_speed_ref = 0
                self.left_speed_ref = 0
                
            
        except:
            self.get_logger().info('Failed to find next speeds in pure pursuit.')
    
    def draw_pursuit(self):
        # Create a circle at the turn center
        circle_marker = Marker()
        circle_marker.header.frame_id = 'odom'
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.ns = 'pursuit'
        circle_marker.id = 0
        circle_marker.type = Marker.CYLINDER
        circle_marker.action = Marker.ADD
        circle_marker.pose.position.x = self.turn_center_x
        circle_marker.pose.position.y = self.turn_center_y
        circle_marker.pose.position.z = 0.0
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = 2 * self.turn_rad  # Diameter
        circle_marker.scale.y = 2 * self.turn_rad
        circle_marker.scale.z = 0.05
        circle_marker.color.a = 0.4  # Transparency
        circle_marker.color.r = 1.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 0.0
        self.pursuit_marker_pub.publish(circle_marker)

        # Publish an arrow at the lookahead point (Lx, Ly)
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'odom'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = 'pursuit'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = self.Lx
        arrow_marker.pose.position.y = self.Ly
        arrow_marker.pose.position.z = 0.5

        # Set the arrow orientation to point downward in Z (quaternion for downward in ROS)
        arrow_marker.pose.orientation.x = 0.0
        arrow_marker.pose.orientation.y = 0.0
        arrow_marker.pose.orientation.z = 0.0
        arrow_marker.pose.orientation.w = 1.0

        arrow_marker.scale.x = 0.0  # Arrow length along X is set to 0 since we point downward
        arrow_marker.scale.y = 0.0  # Set Y and Z to control the arrow's thickness and tip
        arrow_marker.scale.z = 0.5  # The length of the arrow pointing downward

        arrow_marker.color.a = 1.0  # Opaque
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0
        self.pursuit_marker_pub.publish(arrow_marker)
 
        
    def draw_path(self):
        # Publish an orange line with triangle markers for the planned path
        path_marker = Marker()
        path_marker.header.frame_id = 'odom'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 0.65
        path_marker.color.b = 0.0

        # Create a line through the planned path points
        for point in self.planned_path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            path_marker.points.append(p)

        self.path_marker_pub.publish(path_marker)          
    
    def plan_path(self):
        self.astar_plan()
        self.draw_path()
        
        
    def astar_plan(self):
    
        # Get the segment to plan
        self.seg_to_plan()

        # Check the number of poses in the segment
        if self.segment.shape[0] == 2:
            # If there are two poses, plan directly between them
            self.planned_path = self.astar_seg_plan(self.segment[0, :2], self.segment[1, :2], self.odom_plant_map)
        elif self.segment.shape[0] == 3:
            # If there are three poses, plan between the first two and the second two
            path1 = self.astar_seg_plan(self.segment[0, :2], self.segment[1, :2], self.odom_plant_map)
            path2 = self.astar_seg_plan(self.segment[1, :2], self.segment[2, :2], self.odom_plant_map)
            # Combine the paths into one overall path
            self.planned_path = np.vstack([path1, path2[1:, :]])  # Avoid duplicating the middle pose
        #self.get_logger().info(f'{[self.planned_path]}')

    def seg_to_plan(self):
        try:
            distance_to_goal = np.linalg.norm([self.current_goal[0] - self.current_x, self.current_goal[1] - self.current_y])
        
            if distance_to_goal > self.planning_distance:
                direction = (self.current_goal[:2] - np.array([self.current_x, self.current_y])) / distance_to_goal
                segment_end = np.array([self.current_x, self.current_y]) + self.planning_distance * direction
                segment_heading = self.current_heading + (self.current_goal[2] - self.current_heading) * (self.planning_distance / distance_to_goal)
                self.segment = np.array([[self.current_x, self.current_y, self.current_heading], [segment_end[0], segment_end[1], segment_heading]])
            elif self.current_goal_index < self.goal_points_xy.shape[0] - 1:
                next_goal = self.goal_points_xy[self.current_goal_index + 1, :]
                direction = (next_goal[:2] - self.current_goal[:2]) / np.linalg.norm(next_goal[:2] - self.current_goal[:2])
                segment_end = self.current_goal[:2] + (self.planning_distance - distance_to_goal) * direction
                segment_heading = self.current_goal[2] + (next_goal[2] - self.current_goal[2]) * ((self.planning_distance - distance_to_goal) / np.linalg.norm(next_goal[:2] - self.current_goal[:2]))
                self.segment = np.array([[self.current_x, self.current_y, self.current_heading], self.current_goal, [segment_end[0], segment_end[1], segment_heading]])
            else:
                self.segment = np.array([[self.current_x, self.current_y, self.current_heading], self.current_goal])
        except:
            self.get_logger().info('Failed to identify planning segment.')

    def astar_seg_plan(self, start_pose, goal_pose, odom_map):  
        try:
            try:
                #Calculate the direction vector from start to goal
                direction = (goal_pose - start_pose) / np.linalg.norm(goal_pose - start_pose)
                perpendicular = np.array([-direction[1], direction[0]])
            
                #Initialize the central line with the start pose
                X = [start_pose[0]]
                Y = [start_pose[1]]
            
                #Step 1: Move along the direction vector and add grid points
                current_point = start_pose
                while np.linalg.norm(current_point - start_pose) < np.linalg.norm(goal_pose - start_pose):
                    current_point = current_point + self.grid_size * direction
                    X.append(current_point[0])
                    Y.append(current_point[1])
                
                #Step 2: Extend the central axis by padding on both ends
                padding = 0  # Define padding distance

                pre_padding_point = start_pose - self.grid_size * direction
                while np.linalg.norm(pre_padding_point - start_pose) < padding:
                    pre_padding_point = pre_padding_point - self.grid_size * direction       
                    X.insert(0, pre_padding_point[0])
                    Y.insert(0, pre_padding_point[1])

                post_padding_point = current_point + self.grid_size * direction
                while np.linalg.norm(post_padding_point - goal_pose) < padding:
                    post_padding_point = post_padding_point + self.grid_size * direction
                    X.append(post_padding_point[0])
                    Y.append(post_padding_point[1])
                    
                #Step 3: Create grid points perpendicular to the central line
                #Create grid points in the perpendicular direction
                half_width = max(np.linalg.norm(goal_pose - start_pose) / 4, 0.5)
            
                X_full = np.zeros((int(half_width / self.grid_size) + 1, len(X)))
                Y_full = np.zeros((int(half_width / self.grid_size) + 1, len(Y)))
                X_full[0, :] = X
                Y_full[0, :] = Y
                for i in range(1, int(half_width / self.grid_size) + 1):
                    X_full[i, :] = X_full[0, :] + i * self.grid_size * perpendicular[0]
                    Y_full[i, :] = Y_full[0, :] + i * self.grid_size * perpendicular[1]

                X_full2 = np.zeros((int(half_width / self.grid_size), len(X)))
                Y_full2 = np.zeros((int(half_width / self.grid_size), len(Y)))
                for i in range(1, int(half_width / self.grid_size) + 1):
                    X_full2[i - 1, :] = np.array(X) - i * self.grid_size * perpendicular[0]
                    Y_full2[i - 1, :] = np.array(Y) - i * self.grid_size * perpendicular[1]
                
                X = np.vstack([np.flip(X_full2, axis=0), X_full])
                Y = np.vstack([np.flip(Y_full2, axis=0), Y_full])
            except Exception as e:
                self.get_logger().info(f'Exception during grid construction: {e}')

            try:
                #mask the current map to the search space so we don't need to compare to every single plant
                if odom_map is not None and len(odom_map) > 0:
                    min_x = np.min(X)
                    max_x = np.max(X)
                    min_y = np.min(Y)
                    max_y = np.max(Y)
                    
                    inside_mask = (odom_map[:, 0] >= min_x) & (odom_map[:, 0] <= max_x) & \
                                  (odom_map[:, 1] >= min_y) & (odom_map[:, 1] <= max_y)
                              
                    local_map = odom_map[inside_mask, :]
                else:
                    local_map = np.empty((0, 3))
            except Exception as e:
                self.get_logger().info(f'Exception during map masking')
        
            # Initialize the open and closed lists of nodes being explored
            open_list = np.array([[start_pose[0], start_pose[1], self.h_cost(start_pose, start_pose, goal_pose, local_map), 0, self.h_cost(start_pose, start_pose, goal_pose, local_map), np.nan, np.nan]])
            closed_list = np.empty((0, 7))
            path = np.empty((0, 2))

            # A* Loop
            while len(open_list) > 0:
                #find the node with the lower evaluation function f
                idx_to_eval = np.argmin(open_list[:, 4])
                node_to_eval = open_list[idx_to_eval, :]

                # Check if our planning has reached the goal
                if np.linalg.norm(node_to_eval[:2] - goal_pose) < (self.grid_size / 2):
                    path = []
                    current_node = node_to_eval
                    
                    # Construct path by following parent links
                    while not np.isnan(current_node[5]) and not np.isnan(current_node[6]):
                        path.append(current_node[:2])
                        parent_idx = np.where((closed_list[:, 0] == current_node[5]) & (closed_list[:, 1] == current_node[6]))[0][0]
                        current_node = closed_list[parent_idx, :]
                    if len(path) > 0:
                        path.append(start_pose[:2])
                        path = np.flip(path, axis=0)
                    else:
                        path = [start_pose[:2]]
                    break
                
                # Update the open and closed lists
                open_list = np.delete(open_list, idx_to_eval, axis=0)
                closed_list = np.vstack([closed_list, node_to_eval]) 

                # Explore neighboring nodes
                neighbors = np.array([
                    node_to_eval[:2] + self.grid_size * direction,  # Move in the direction of the path
                    node_to_eval[:2] - self.grid_size * direction,  # Move in the opposite direction
                    node_to_eval[:2] + self.grid_size * perpendicular,  # Move perpendicular to the path
                    node_to_eval[:2] - self.grid_size * perpendicular  # Move in the opposite perpendicular direction
                ])

                tolerance = 1e-8
                
                for neighbor in neighbors:
                    is_within_bounds = np.any(np.all(np.abs(np.array([X.flatten(), Y.flatten()]).T - neighbor) < tolerance, axis=1))
        
                    if is_within_bounds:
                        # Calculate the evaluation function and hueristic functions for the nodes
                        g_cost_neighbor = self.g_cost(node_to_eval[3], self.grid_size)
                        h_cost_neighbor = self.h_cost(neighbor, start_pose, goal_pose, local_map)
                        f_cost_neighbor = g_cost_neighbor + h_cost_neighbor
                        
                        # Check if the neighbor is in the closed list
                        in_closed_list = np.any(np.all(np.abs(closed_list[:, :2] - neighbor) < tolerance, axis=1))
                        if in_closed_list:
                            continue
                        
                        #Check if the neighbor is in the open list
                        in_open_list = np.any(np.all(np.abs(open_list[:, :2] - neighbor) < tolerance, axis=1))
                        if in_open_list:
                            existing_node_idx = np.where(np.all(np.abs(open_list[:, :2] - neighbor) < tolerance, axis=1))[0][0]
                            if f_cost_neighbor < open_list[existing_node_idx, 4]:
                                open_list[existing_node_idx, :] = np.hstack([neighbor, h_cost_neighbor, g_cost_neighbor, f_cost_neighbor, node_to_eval[:2]])
                        else: #if it isn't in the open list, add it to the open list
                            open_list = np.vstack([open_list, np.hstack([neighbor, h_cost_neighbor, g_cost_neighbor, f_cost_neighbor, node_to_eval[:2]])])
            
            if len(path) == 0:
                self.get_logger().info('No current path')
            return np.array(path)
        except:
            self.get_logger().info('A* planning failed.')

    def h_cost(self, current_pose, start_pose, goal_pose, local_map):
        try:
            distance_cost = np.linalg.norm(current_pose - goal_pose) 
            plant_cost = self.h_plant_cost(current_pose, local_map) 
            row_cost = self.h_row_cost(current_pose, local_map)
            robot_turn_cost_at_start_cost = self.start_area_cost(current_pose, start_pose)
            return distance_cost + plant_cost + row_cost + robot_turn_cost_at_start_cost  
        except Exception as e:
            self.get_logger().info(f'Exception as {e} during hueristic function calculation.')
    
    def h_plant_cost(self, point, local_map):
        try:
            cost = 0 
            for i in range(local_map.shape[0]):
                dist_to_obstacle = np.linalg.norm(point - local_map[i, :2])
                if dist_to_obstacle < self.grid_size + 0.01:
                    cost += 70
                else:
                    decay_factor = np.exp(self.obstactle_decay_buffer * dist_to_obstacle)
                    cost += 70 * decay_factor
            return cost
        except Exception as e:
            self.get_logger().info(f'Exception as {e} during plant heuristic eval.')

    def start_area_cost(self, current_pose, start_pose):
        try:
            heading = np.radians(self.current_heading)
            angle_to_point = np.arctan2(current_pose[1] - start_pose[1], current_pose[0] - start_pose[0])
            relative_angle = np.mod(angle_to_point - heading + np.pi, 2 * np.pi) - np.pi
            cup_angle_range = np.radians(300)
            distance = np.sqrt((current_pose[0] - start_pose[0])**2 + (current_pose[1] - start_pose[1])**2)
            if np.abs(relative_angle) > (np.pi - cup_angle_range / 2) and distance < 0.5:
                return 70
            return 0
        except Exception as e:
            self.get_logger().info(f'Exception as {e} during non-holonomic start heuristic.')

    def h_row_cost(self, current_pose, local_map):
        try:
            cost = 0 
            if local_map.shape[0] == 0:
                return cost
            unique_rows = np.unique(local_map[:, 2])
            
            for row in unique_rows:
                row_idx = np.where(local_map[:, 2] == row)[0]
                if len(row_idx) > 1:
                    for j in range(len(row_idx) - 1):
                        plant1 = local_map[row_idx[j], :2]    
                        plant2 = local_map[row_idx[j + 1], :2]
                        
                        num_points = 10  
                        line_points = np.array([np.linspace(plant1[0], plant2[0], num_points), np.linspace(plant1[1], plant2[1], num_points)]).T
                        
                        for point in line_points:
                            dist_to_line = np.linalg.norm(point - np.array([current_pose[0], current_pose[1]]))
                            if dist_to_line < self.grid_size:
                                cost += 70
            return cost
        except Exception as e:
            self.get_logger().info(f'Exception as {e} during row gap heuristic eval.')

    def g_cost(self, parent_g_cost, grid_size):
        return parent_g_cost + grid_size

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    time.sleep(0.5)
    
    try:    
        rclpy.spin(node)
    
    except Exception as e:
        node.get_logger().info(f'Exception as {e} during node execution. Shutting down node...')
        node.shutdown_robot()           


if __name__ == '__main__':
    main()

