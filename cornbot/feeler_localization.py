import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, TransformStamped, PointStamped
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from tf2_ros import TransformListener
from tf2_ros import Buffer
import math

class FeelerLocalization(Node):
    def __init__(self):
        super().__init__('feeler_localization')
        self.get_logger().info("Feeler Localization Started")

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for left-related topics
        self.left_contact_sub = Subscriber(self, QuaternionStamped, 'cornbot/feeler/left_contact_state_numerical')
        self.left_state_sub = Subscriber(self, QuaternionStamped, 'cornbot/feeler/left_state')

        # Subscribers for right-related topics
        self.right_contact_sub = Subscriber(self, QuaternionStamped, 'cornbot/feeler/right_contact_state_numerical')
        self.right_state_sub = Subscriber(self, QuaternionStamped, 'cornbot/feeler/right_state')

        # Subscriber for wheel speed reference
        self.wheel_speed_ref_sub = self.create_subscription(String, 'cornbot/speed_ref_topic', self.speed_ref_callback, 1)

        # Synchronizer for left-related topics
        ats_left = ApproximateTimeSynchronizer([self.left_contact_sub, self.left_state_sub], queue_size=10, slop=0.1)
        ats_left.registerCallback(self.left_synchronized_callback)

        # Synchronizer for right-related topics
        ats_right = ApproximateTimeSynchronizer([self.right_contact_sub, self.right_state_sub], queue_size=10, slop=0.1)
        ats_right.registerCallback(self.right_synchronized_callback)

        # Publishers for each frame
        self.sensor_frame_pub = self.create_publisher(PointStamped, '/cornbot/plant_localization/sensor_frame', 10)
        self.robot_frame_pub = self.create_publisher(PointStamped, '/cornbot/plant_localization/robot_frame', 10)
        self.global_frame_pub = self.create_publisher(PointStamped, '/cornbot/plant_localization/global_frame', 10)
        self.marker_pub = self.create_publisher(Marker, '/cornbot/plant_marker', 10)

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

    def speed_ref_callback(self, msg):
        speeds = msg.data.strip('<>').split(',')
        self.right_speed = float(speeds[0])
        self.left_speed = float(speeds[1])

    def left_synchronized_callback(self, contact_msg, state_msg):
        self.left_contact_state = contact_msg.quaternion.x
        self.left_angle = state_msg.quaternion.x
        self.left_angle_history.append(self.left_angle)

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
            mount_angle=math.radians(90)
            angle_history = self.left_angle_history
            y_del_t = self.left_y_del_t
            sensor_frame = "left_sensor"
        else:
            contact_state = self.right_contact_state
            angle = self.right_angle
            mount_angle=math.radians(-90)
            angle_history = self.right_angle_history
            y_del_t = self.right_y_del_t
            sensor_frame = "right_sensor"

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
                    if abs(self.H_hat) > self.paddle_length + 0.01 or self.D_hat<0.0:
                        self.D_hat = float('nan')
                        self.P_hat = float('nan')
                        self.H_hat = float('nan')
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
                    # Transformation from robot base to the global frame
                    trans_global = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())  # Using the latest available time

                    # Final transformation to global frame
                    plant_in_global_frame = [trans_global.transform.translation.x + plant_in_robot_frame[0],
                                             trans_global.transform.translation.y + plant_in_robot_frame[1],
                                             1]

                    # Publish the points in each frame
                    self.publish_point(sensor_frame, plant_localization_in_sensor_frame, stamp)
                    self.publish_point("base_link", plant_in_robot_frame, stamp)
                    self.publish_point("odom", plant_in_global_frame, stamp)

                    # Publish the marker for RViz visualization
                    self.publish_plant_marker(plant_in_global_frame, "odom", stamp)

                except tf2_ros.LookupException:
                    self.get_logger().error("Transform lookup failed")

            else:
                # Store NaN when the estimation cannot be performed
                self.D_hat = float('nan')
                self.P_hat = float('nan')
                self.H_hat = float('nan')
                plant_localization_in_sensor_frame = [float('nan'), float('nan'), float('nan')]
                plant_in_robot_frame = [float('nan'), float('nan'), float('nan')]
                plant_in_global_frame = [float('nan'), float('nan'), float('nan')]

        else:
            # Store NaN when not in contact
            self.D_hat = float('nan')
            self.P_hat = float('nan')
            self.H_hat = float('nan')
            plant_localization_in_sensor_frame = [float('nan'), float('nan'), float('nan')]
            plant_in_robot_frame = [float('nan'), float('nan'), float('nan')]
            plant_in_global_frame = [float('nan'), float('nan'), float('nan')]

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
            self.global_frame_pub.publish(point_msg)

    def publish_plant_marker(self, plant_position, frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "plants"
        marker.id = int(stamp.nanosec)  # Unique ID based on timestamp
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = plant_position[0]
        marker.pose.position.y = plant_position[1]
        marker.pose.position.z = 1.5  # Center of the cylinder (half of the height)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Diameter of the cylinder
        marker.scale.y = 0.1  # Diameter of the cylinder
        marker.scale.z = 3.0  # Height of the cylinder
        marker.color.a = 1.0  # Alpha (opacity)
        marker.color.r = 0.0
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

