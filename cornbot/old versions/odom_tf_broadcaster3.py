import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped, Quaternion
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf2_ros
import math

class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('robot_odom_broadcaster')

        # Initial position and heading
        self.initial_lat = None
        self.initial_lon = None
        self.current_x = None
        self.current_y = None
        self.current_heading = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF Buffer and Listener for transforming IMU data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for GNSS data
        self.create_subscription(PointStamped, 'gnss/positioning/lat_lon_stamped_topic', self.update_position_callback, 1)
        # IMU subscriber
        self.create_subscription(Imu, 'imu/data', self.update_imu_callback, 10)

        # Publisher for the robot's current pose
        self.pose_pub = self.create_publisher(PoseStamped, 'gnss/tf_pose', 10)

        # Publisher for the initial lat/lon values
        self.startup_pose_pub = self.create_publisher(PointStamped, 'gnss/startup_pose', 10)

        self.timer = self.create_timer(0.001, self.broadcast_transform)

    def update_position_callback(self, msg):
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

        self.current_x, self.current_y = self.latlon_to_xy(self.initial_lat, self.initial_lon, msg.point.x, msg.point.y)

    def update_imu_callback(self, msg: Imu):
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        print(f'Roll: {math.degrees(roll):.2f}, Pitch: {math.degrees(pitch):.2f}, Yaw: {math.degrees(yaw):.2f}')

        try:
            corrected_quaternion = Quaternion(
                    x=-msg.orientation.x,
                    y=msg.orientation.y,
		            z=msg.orientation.z,
                    w=msg.orientation.w
         	)           	
		
            self.current_heading = corrected_quaternion

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to use IMU data: {e}")

    def broadcast_transform(self):
        if self.current_x is not None and self.current_y is not None:
            if self.current_heading is None:
                self.current_heading = Quaternion()

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.translation.z = 0.0762

            # Apply the current_heading directly from the IMU
            t.transform.rotation = self.current_heading

            self.tf_broadcaster.sendTransform(t)

            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = self.current_x
            pose_msg.pose.position.y = self.current_y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation = self.current_heading

            self.pose_pub.publish(pose_msg)

            # Also republish the initial lat/lon values to keep them available
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
   
    def quaternion_to_euler(self, x, y, z, w):     
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        Roll is rotation around x in radians (counterclockwise)
        Pitch is rotation around y in radians (counterclockwise)
        Yaw is rotation around z in radians (counterclockwise)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

