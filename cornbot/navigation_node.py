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
        self.fix_string = 'Fix Quality Unknown'
        self.goal_points_latlon = [
            {"lat": 43.70409011798969, "lon": -72.29561185773956, "angle": 180.001},
            {"lat": 43.704025196022116, "lon": -72.29592550489402, "angle": 180.001}
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
        
        #Broadcast so we see changes in RViz
        self.timer = self.create_timer(0.008, self.broadcast_transform)
        
        #client side for us to request the current map from the feeler localization node
        self.map_service_client = self.create_client(Trigger, '/odom_plant_map')
        while not self.map_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Plant map not available. Waiting...')

        self.odom_plant_map=[]
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
     
    def cleanupgoalpoint():
        pass
    
           
    def plan_path(self):
        pass

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
            #self.publish_ref_speed(-0.1, -0.1)
            return
           
            
        #convert our lat lon goals to x y goals w.r.t. start location lat lon
        for i in range(len(self.goal_points_latlon)):
            local_coords = self.latlon_to_xy(self.initial_lat, self.initial_lon, self.goal_points_latlon[i]["lat"], self.goal_points_latlon[i]["lon"])
            goal_point = [local_coords[0], local_coords[1], self.goal_points_latlon[i]["angle"]]
            self.goal_points_xy.append(goal_point)
            #self.get_logger().info(f'Goal point {i} is ~{goal_point} meters away (+x=east, +y=north, yaw=RHR from x)')

        self.goal_points_xy = np.array(self.goal_points_xy)
        
        self.cleanup_goal_points()
        self.get_logger().info(f'Processed xy goal list: {self.goal_points_xy})')
        self.display_goal_markers()
        #self.plan_plath()
        
    def display_goal_markers(self):
        for i in range(len(self.goal_points_xy)):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_points"
            marker.id = i # Use the provided marker_id for consistent identification
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
            if self.current_lat is None or self.current_lon is None or self.initial_lat is None or self.current_heading is None or self.goal_points_xy==[]:
                self.init_nav()
                
            else:   #we will cancel the startup timer, stop the robot and finally start the main loop timer.
                self.publish_ref_speed(0.0, 0.0)
                self.get_logger().info(f'Initialized lat, lon: {(self.initial_lat, self.initial_lon)}')
                self.get_logger().info(f'Initialized heading: {self.current_heading} deg.')
                self.get_logger().info('Finalized startup.')
                self.start_up_timer.cancel()
                
        except Exception as e:
            self.get_logger().info(f'Exception during GNSS startup execution as {e}. Shutting down node...')
            self.shutdown_robot()
            
    
            
    

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

