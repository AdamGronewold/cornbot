import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from tf2_ros import TransformListener, Buffer
import math
import numpy as np
from std_srvs.srv import Trigger
import json


class FeelerLocalization(Node):
    def __init__(self):
        super().__init__('feeler_localization')
        self.get_logger().info("Feeler Localization Started")

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for left-related topics
        self.left_contact_sub = Subscriber(self, QuaternionStamped, 'feeler/left_contact_state_numerical')
        self.left_state_sub = Subscriber(self, QuaternionStamped, 'feeler/left_state')

        # Subscribers for right-related topics
        self.right_contact_sub = Subscriber(self, QuaternionStamped, 'feeler/right_contact_state_numerical')
        self.right_state_sub = Subscriber(self, QuaternionStamped, 'feeler/right_state')

        # Subscriber for wheel speed reference
        self.wheel_speed_ref_sub = self.create_subscription(String, 'speed_ref_topic', self.speed_ref_callback, 1)

        # Synchronizer for left-related topics
        ats_left = ApproximateTimeSynchronizer([self.left_contact_sub, self.left_state_sub], queue_size=10, slop=0.1)
        ats_left.registerCallback(self.left_synchronized_callback)

        # Synchronizer for right-related topics
        ats_right = ApproximateTimeSynchronizer([self.right_contact_sub, self.right_state_sub], queue_size=10, slop=0.1)
        ats_right.registerCallback(self.right_synchronized_callback)

        # Publishers for each frame
        self.sensor_frame_pub = self.create_publisher(PointStamped, 'plants/localization/sensor_frame', 10)
        self.robot_frame_pub = self.create_publisher(PointStamped, 'plants/localization/robot_frame', 10)
        self.odom_frame_pub = self.create_publisher(PointStamped, 'plants/localization/odom_frame', 10)
        self.map_update_pub = self.create_publisher(PointStamped, 'plants/mapping/odom_frame', 10)
        self.marker_pub = self.create_publisher(Marker, '/plant_marker', 100)

        # Service for map requests
        self.map_service = self.create_service(Trigger, '/odom_plant_map', self.handle_get_map_request)

        
        # Initializations
        self.left_contact_state = 0.0
        self.left_angle = 0.0
        self.right_contact_state = 0.0
        self.right_angle = 0.0
        self.left_angle_history = []
        self.right_angle_history = []
        self.left_time = None
        self.right_time = None
        self.track_width = 0.4064  # Robot track width in meters
        self.paddle_length = 0.3556
        self.right_speed = 0.0
        self.left_speed = 0.0
        self.left_y_del_t = 0.008  # the sample period, which changes over time
        self.right_y_del_t = 0.008

        # Initialize history buffers
        self.left_contact_history = []
        self.right_contact_history = []

        # Initialize plant localization history in odom frame
        self.left_plant_in_odom_frame_history = []
        self.right_plant_in_odom_frame_history = []
        self.odom_plant_map = []
        self.row_number = 1

    def speed_ref_callback(self, msg):
        speeds = msg.data.strip('<>').split(',')
        self.right_speed = float(speeds[0])
        self.left_speed = float(speeds[1])

    def left_synchronized_callback(self, contact_msg, state_msg):
        self.left_contact_state = contact_msg.quaternion.x
        self.left_angle = state_msg.quaternion.x
        
        self.left_angle_history.append(self.left_angle)
        self.left_contact_history.append(self.left_contact_state)
        if len(self.left_contact_history) > 200:
            self.left_contact_history.pop(0)

        # Calculate the time difference from the previous message
        current_time = contact_msg.header.stamp.sec + contact_msg.header.stamp.nanosec / 1e9
        if self.left_time is not None:
            self.left_y_del_t = current_time - self.left_time
        self.left_time = current_time

        self.estimate_values("left", contact_msg.header.stamp)

    def right_synchronized_callback(self, contact_msg, state_msg):
        self.right_contact_state = contact_msg.quaternion.x
        self.right_angle = state_msg.quaternion.x
        self.right_angle_history.append(self.right_angle)
        
        self.right_contact_history.append(self.right_contact_state)
        if len(self.right_contact_history) > 200:
            self.right_contact_history.pop(0)

        # Calculate the time difference from the previous message
        current_time = contact_msg.header.stamp.sec + contact_msg.header.stamp.nanosec / 1e9
        if self.right_time is not None:
            self.right_y_del_t = current_time - self.right_time
        self.right_time = current_time

        self.estimate_values("right", contact_msg.header.stamp)

    def estimate_values(self, side, stamp):
        if side == "left":
            contact_state = self.left_contact_state
            angle = self.left_angle
            mount_angle = math.radians(90)
            angle_history = self.left_angle_history
            y_del_t = self.left_y_del_t
            sensor_frame = "left_sensor"
            plant_in_odom_frame_history = self.left_plant_in_odom_frame_history
        else:
            contact_state = self.right_contact_state
            angle = self.right_angle
            mount_angle = math.radians(-90)
            angle_history = self.right_angle_history
            y_del_t = self.right_y_del_t
            sensor_frame = "right_sensor"
            plant_in_odom_frame_history = self.right_plant_in_odom_frame_history

        # Here you have your conditions to perform the calculations
        if (contact_state == 80.0 or contact_state == 90.0) and len(angle_history) > 1 and y_del_t is not None:
            theta_k = math.radians(angle)
            theta_k_minus_1 = math.radians(angle_history[-2])
            cos_theta_k = math.cos(theta_k)
            cos_theta_k_minus_1 = math.cos(theta_k_minus_1)
            sin_theta_diff = math.sin(theta_k - theta_k_minus_1)

            if sin_theta_diff != 0:
                omega = (self.right_speed - self.left_speed) / self.track_width

                # Position of the sensor relative to the robot's center (in robot frame)
                try:
                    trans = self.tf_buffer.lookup_transform('base_link', sensor_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))

                    # Translation components
                    d_x = trans.transform.translation.x
                    d_y = trans.transform.translation.y

                    # Calculate the velocity of the sensor in the robot frame
                    v_robot = [(self.right_speed + self.left_speed) / 2, 0]  # Average speed in x direction
                    omega_cross_d = [-omega * d_y, omega * d_x]  # Rotational component

                    v_sensor_x = v_robot[0] + omega_cross_d[0]  # X-component of the velocity

                    # Speed estimate for localization
                    speed_estimate = 0.2  # Placeholder; replace with -v_sensor_x for actual use

                    # Estimation of D, P, and H using the updated speed estimate
                    D_hat = abs(speed_estimate * y_del_t * cos_theta_k * cos_theta_k_minus_1) / abs(sin_theta_diff)
                    P_hat_k = D_hat * math.tan(theta_k)
                    H_hat_k = D_hat / math.cos(theta_k)

                    # Store estimates
                    self.D_hat = D_hat
                    self.P_hat = P_hat_k
                    self.H_hat = H_hat_k

                    # Check if H_hat exceeds the paddle length
                    if abs(self.H_hat) > self.paddle_length + 0.01 or self.D_hat < 0.0:
                        self.D_hat = float('nan')
                        self.P_hat = float('nan')
                        self.H_hat = float('nan')
                        plant_localization_in_sensor_frame = [float('nan'), float('nan'), float('nan')]
                        plant_in_robot_frame = [float('nan'), float('nan'), float('nan')]
                        plant_in_odom_frame = [float('nan'), float('nan'), float('nan')]
                        if side == "left":
                            self.left_plant_in_odom_frame_history.append(plant_in_odom_frame)
                            if len(self.left_plant_in_odom_frame_history) > 200:
                                self.left_plant_in_odom_frame_history.pop(0)
                        else:
                            self.right_plant_in_odom_frame_history.append(plant_in_odom_frame)
                            if len(self.right_plant_in_odom_frame_history) > 200:
                                self.right_plant_in_odom_frame_history.pop(0)
                        return

                    # Localization in the sensor frame
                    plant_localization_in_sensor_frame = [self.D_hat, self.P_hat, 1]
                    T = [
                        [math.cos(mount_angle), -math.sin(mount_angle), d_x],
                        [math.sin(mount_angle), math.cos(mount_angle), d_y],
                        [0, 0, 1]
                    ]
                    plant_in_robot_frame = [
                        T[0][0] * plant_localization_in_sensor_frame[0] + T[0][1] * plant_localization_in_sensor_frame[1] + T[0][2],
                        T[1][0] * plant_localization_in_sensor_frame[0] + T[1][1] * plant_localization_in_sensor_frame[1] + T[1][2],
                        1
                    ]
                    # Transformation from robot base to the odom frame
                    trans_odom = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())  # Using the latest available time

                    # Extract the yaw angle from the quaternion (assuming planar motion)
                    qx = trans_odom.transform.rotation.x
                    qy = trans_odom.transform.rotation.y
                    qz = trans_odom.transform.rotation.z
                    qw = trans_odom.transform.rotation.w

                    # Convert quaternion to a yaw angle (assuming no roll or pitch)
                    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

                    # Construct the rotation matrix
                    T_odom = [
                        [math.cos(yaw), -math.sin(yaw), trans_odom.transform.translation.x],
                        [math.sin(yaw), math.cos(yaw), trans_odom.transform.translation.y],
                        [0, 0, 1]
                    ]

                    # Apply the transformation from robot frame to odom frame
                    plant_in_odom_frame = [
                        T_odom[0][0] * plant_in_robot_frame[0] + T_odom[0][1] * plant_in_robot_frame[1] + T_odom[0][2],
                        T_odom[1][0] * plant_in_robot_frame[0] + T_odom[1][1] * plant_in_robot_frame[1] + T_odom[1][2],
                        1
                    ]

                    # Append to history and manage size
                    if side == "left":
                        self.left_plant_in_odom_frame_history.append(plant_in_odom_frame)
                        if len(self.left_plant_in_odom_frame_history) > 200:
                            self.left_plant_in_odom_frame_history.pop(0)
                    else:
                        self.right_plant_in_odom_frame_history.append(plant_in_odom_frame)
                        if len(self.right_plant_in_odom_frame_history) > 200:
                            self.right_plant_in_odom_frame_history.pop(0)

                    # Publish the points in each frame
                    self.publish_point(sensor_frame, plant_localization_in_sensor_frame, stamp)
                    self.publish_point("base_link", plant_in_robot_frame, stamp)
                    self.publish_point("odom", plant_in_odom_frame, stamp)
                    self.add_to_odom_map(side, stamp)

                except tf2_ros.LookupException:
                    self.get_logger().error("Transform lookup failed")

            else:
                # Store NaN when the estimation cannot be performed
                self.D_hat = float('nan')
                self.P_hat = float('nan')
                self.H_hat = float('nan')
                plant_localization_in_sensor_frame = [float('nan'), float('nan'), float('nan')]
                plant_in_robot_frame = [float('nan'), float('nan'), float('nan')]
                plant_in_odom_frame = [float('nan'), float('nan'), float('nan')]
                if side == "left":
                    self.left_plant_in_odom_frame_history.append(plant_in_odom_frame)
                    if len(self.left_plant_in_odom_frame_history) > 200:
                        self.left_plant_in_odom_frame_history.pop(0)
                else:
                    self.right_plant_in_odom_frame_history.append(plant_in_odom_frame)
                    if len(self.right_plant_in_odom_frame_history) > 200:
                        self.right_plant_in_odom_frame_history.pop(0)
        else:
            # Store NaN when not in contact
            self.D_hat = float('nan')
            self.P_hat = float('nan')
            self.H_hat = float('nan')
            plant_localization_in_sensor_frame = [float('nan'), float('nan'), float('nan')]
            plant_in_robot_frame = [float('nan'), float('nan'), float('nan')]
            plant_in_odom_frame = [float('nan'), float('nan'), float('nan')]
            if side == "left":
                self.left_plant_in_odom_frame_history.append(plant_in_odom_frame)
                if len(self.left_plant_in_odom_frame_history) > 200:
                    self.left_plant_in_odom_frame_history.pop(0)
            else:
                self.right_plant_in_odom_frame_history.append(plant_in_odom_frame)
                if len(self.right_plant_in_odom_frame_history) > 200:
                    self.right_plant_in_odom_frame_history.pop(0)

    def add_to_odom_map(self, side, stamp):
        if side == "left":
            plant_in_odom_history = self.left_plant_in_odom_frame_history
            contact_history = self.left_contact_history
            sensor_frame="left_sensor"
        else:
            plant_in_odom_history = self.right_plant_in_odom_frame_history
            contact_history = self.right_contact_history
            sensor_frame="right_sensor"
            
        row_threshold = 1  # Threshold distance to consider the same row, in meters

        # Sensor tag based on the provided sensor index
        sensor_tag = side

        contact_states = [80.0, 90.0]  # In Contact = 80.0, Nearing Crash = 90.0

        start_idx = None
        end_idx = None
    
        # Iterate backwards to find the last transition into a contact state
        for i in range(len(contact_history) - 1, 1, -1):
            if contact_history[i] in contact_states and contact_history[i - 1] not in contact_states:
                start_idx = i
                break
                
        #if after looping through everthing we haven't found a transition, set the start index to the first value. If the contact history at that value is not in contact, there is no contact to map, return
        if start_idx is None:
            start_idx=0
            if contact_history[start_idx] not in contact_states:
                self.get_logger().info('Return from here')
                return
                
        #set the end index to the final contact state. If the final index is not in contact and localizations in the plant_in_odom_history have already been mapped, so return.       
        end_idx = len(contact_history) - 1
        if contact_history[end_idx] not in contact_states:
            self.get_logger().info('Return from there.')
            return

        #reduce noise in the mapping from short contact periods
        if end_idx - start_idx < 4:
            return
            
        #self.get_logger().info(f'Start idx:{start_idx}, End idx:{end_idx}')

        if start_idx is not None and end_idx is not None and start_idx < end_idx:
            # Extract the localization data during the contact period
            valid_localizations = plant_in_odom_history[start_idx:end_idx]
            
            # Filter out NaN values and calculate the average of non-NaN values for x and y
            x_values = [loc[0] for loc in valid_localizations if not math.isnan(loc[0])]
            y_values = [loc[1] for loc in valid_localizations if not math.isnan(loc[1])]

            # Calculate current point based on non-NaN values
            if len(x_values) > 0 and len(y_values) > 0:
                current_point = [
                    sum(x_values) / len(x_values),  # Average of non-NaN x values
                    sum(y_values) / len(y_values),  # Average of non-NaN y values
                    1  # Set z to 1
                ]
            else:
                return

            #self.get_logger().info(f'{self.odom_plant_map}') 
            
            # Check if there is another point within 0.13 meters
            #self.get_logger().info(f'{len(self.odom_plant_map)}')
            if len(self.odom_plant_map) > 0:
                distances = [math.sqrt((map_point[0] - current_point[0]) ** 2 + (map_point[1] - current_point[1]) ** 2)
                             for map_point in self.odom_plant_map]
                #self.get_logger().info(f'{distances}')
                close_point_idx = next((i for i, dist in enumerate(distances) if dist < 0.13), None)
            else:
                close_point_idx = None
            #self.get_logger().info(f'{close_point_idx}')
            if close_point_idx is None:
                # No close point, add as new entry with a new marker ID
                new_marker_id = len(self.odom_plant_map)
                               
                # Compute Euclidean distances from current point to all points in the global map
                if len(self.odom_plant_map) > 0:
                    distances = [math.sqrt((map_point[0] - current_point[0]) ** 2 + (map_point[1] - current_point[1]) ** 2)
                                 for map_point in self.odom_plant_map]
                    min_distance = np.min(distances)
                    min_idx = np.argmin(distances)

                    # Determine the row number based on the closest point and its sensor tag
                    
                    if min_distance < row_threshold and self.odom_plant_map[min_idx][3] == sensor_tag:
                        self.row_number = self.odom_plant_map[min_idx][2]  # Assign to existing row
                    else:
                        self.row_number += 1  # Create a new row
                    
                self.odom_plant_map.append([current_point[0], current_point[1], self.row_number, sensor_tag, new_marker_id])
                
                map_msg = PointStamped()
                map_msg.header.stamp = stamp
                map_msg.header.frame_id = sensor_tag
                map_msg.point.x = current_point[0]
                map_msg.point.y = current_point[1]
                map_msg.point.z = float(self.row_number)
                self.map_update_pub.publish(map_msg)

                # Publish the marker with the new marker ID
                self.publish_plant_marker(current_point, sensor_frame, stamp, new_marker_id)
                
            else:
                # Update the existing point and marker
                
                self.odom_plant_map[close_point_idx][0] = (self.odom_plant_map[close_point_idx][0] + current_point[0]) / 2
                self.odom_plant_map[close_point_idx][1] = (self.odom_plant_map[close_point_idx][1] + current_point[1]) / 2
                self.odom_plant_map[close_point_idx][2] = self.odom_plant_map[close_point_idx][2]  # Maintain the existing row number
                self.odom_plant_map[close_point_idx][3] = sensor_tag  # Update the sensor tag
                marker_id = self.odom_plant_map[close_point_idx][4]  # Use the existing marker ID

                map_msg = PointStamped()
                map_msg.header.stamp = stamp
                map_msg.header.frame_id = sensor_tag
                map_msg.point.x = self.odom_plant_map[close_point_idx][0]
                map_msg.point.y = self.odom_plant_map[close_point_idx][1]
                map_msg.point.z = float(self.odom_plant_map[close_point_idx][2])
                self.map_update_pub.publish(map_msg)

                # Update the marker with the same marker ID
                self.publish_plant_marker(self.odom_plant_map[close_point_idx][:2], "odom", stamp, marker_id)

    def handle_get_map_request(self, request, response):
        try:
            # Serialize the map to a JSON string
            map_str = json.dumps([(round(x, 4), round(y, 4), row, sensor, marker) for (x, y, row, sensor, marker) in self.odom_plant_map])
            response.success = True
            response.message = map_str
        except Exception as e:
            self.get_logger().error(f"Failed to serialize map: {str(e)}")
            response.success = False
            response.message = str(e)
        return response

    
    def publish_point(self, frame_id, point, stamp):
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = frame_id
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = 0.0

        if frame_id == "left_sensor" or frame_id == "right_sensor":
            self.sensor_frame_pub.publish(point_msg)
        elif frame_id == "base_link":
            self.robot_frame_pub.publish(point_msg)
        elif frame_id == "odom":
            self.odom_frame_pub.publish(point_msg)

    def publish_plant_marker(self, plant_position, frame_id, stamp, marker_id):
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "plants"
        marker.id = marker_id  # Use the provided marker_id for consistent identification
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = plant_position[0]
        marker.pose.position.y = plant_position[1]
        marker.pose.position.z = 1.5  # Center of the cylinder (half of the height)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03  # Diameter of the cylinder
        marker.scale.y = 0.03  # Diameter of the cylinder
        marker.scale.z = 3.0  # Height of the cylinder
        marker.color.a = 1.0  # Alpha (opacity)
        marker.color.r = 0.1
        marker.color.g = 1.0  # Green color
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FeelerLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

