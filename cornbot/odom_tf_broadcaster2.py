import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import math

class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Initial position and heading
        self.initial_lat = None
        self.initial_lon = None
        self.current_x = None
        self.current_y = None
        self.current_heading = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers for GNSS data
        self.create_subscription(PointStamped, 'cornbot/gnss/positioning/lat_lon_stamped_topic', self.update_position_callback, 1)
        self.create_subscription(PointStamped, 'cornbot/gnss/positioning/course_over_ground', self.update_heading_callback, 1)

        # Publisher for the robot's current pose
        self.pose_pub = self.create_publisher(PoseStamped, 'cornbot/gnss/tf_pose', 10)

        # Publisher for the initial lat/lon values
        self.startup_pose_pub = self.create_publisher(PointStamped, 'cornbot/gnss/startup_pose', 10)

        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def update_position_callback(self, msg):
        if self.initial_lat is None and self.initial_lon is None:
            self.initial_lat = msg.point.x
            self.initial_lon = msg.point.y

            # Publish the initial lat/lon values
            startup_pose_msg = PointStamped()
            startup_pose_msg.header.stamp = self.get_clock().now().to_msg()
            startup_pose_msg.point.x = self.initial_lat
            startup_pose_msg.point.y = self.initial_lon
            self.startup_pose_pub.publish(startup_pose_msg)

        self.current_x, self.current_y = self.latlon_to_xy(self.initial_lat, self.initial_lon, msg.point.x, msg.point.y)

    def update_heading_callback(self, msg):
        self.current_heading = msg.point.x
        self.current_heading = (90 - self.current_heading) % 360

    def broadcast_transform(self):
        if self.current_x is not None and self.current_y is not None and self.current_heading is not None:
            # Create TransformStamped message
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.translation.z = 0.0

            q = self.euler_to_quaternion(0, 0, math.radians(self.current_heading))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)

            # Publish the current pose
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

