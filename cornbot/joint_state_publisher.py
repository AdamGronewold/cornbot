import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped, Point
import math

class WheelAndSensorJointPublisher(Node):
    def __init__(self):
        super().__init__('wheel_and_sensor_joint_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.feeler_sub = self.create_subscription(
            QuaternionStamped, 'cornbot/feeler/all/angles', self.feeler_angle_callback,  1)
        self.wheel_speed_sub = self.create_subscription(PointStamped, 'cornbot/wheel_rpm_left_right', self.wheel_rpm_callback, 1)
        
        self.last_time=self.get_clock().now()
        self.left_wheel_angle=0.0
        self.right_wheel_angle=0.0
    
    def feeler_angle_callback(self, msg):
        
        joint_state = JointState()

        # Set the header
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Define joint names and their corresponding angles
        joint_state.name = [
            'left_sensor_joint',  # Added left sensor joint
            'right_sensor_joint'  # Added right sensor joint
        ]

        joint_state.position = [
            math.radians(msg.quaternion.x),         # Left sensor joint angle set to 0
            math.radians(msg.quaternion.y)          # Right sensor joint angle set to 0
        ]

        # Publish the joint state
        self.publisher.publish(joint_state)

    def wheel_rpm_callback(self, msg):
        joint_state = JointState()

        # Set the header
        joint_state.header.stamp = self.get_clock().now().to_msg()
        dt=(self.get_clock().now()-self.last_time).nanoseconds / 1e9
        self.last_time=self.get_clock().now()
        # Define joint names and their corresponding angles
        joint_state.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            #'left_sensor_joint',  # Added left sensor joint
            #'right_sensor_joint'  # Added right sensor joint
        ]
        self.left_wheel_angle=self.left_wheel_angle+dt*(2*math.pi/60)*msg.point.x
        self.right_wheel_angle=self.right_wheel_angle+dt*(2*math.pi/60)*msg.point.y
        
        joint_state.position = [
            self.left_wheel_angle,  # Front left wheel
            self.right_wheel_angle,  # Front right wheel
            self.left_wheel_angle,  # Rear left wheel
            self.right_wheel_angle  # Rear right wheel
        ]

        #joint_state.velocity = [
        #    (2*math.pi/60)*msg.point.x,  # Front left wheel
        #    (2*math.pi/60)*msg.point.y,  # Front right wheel
        #    (2*math.pi/60)*msg.point.x,  # Rear left wheel
        #    (2*math.pi/60)*msg.point.y,  # Rear right wheel
        #]

        # Publish the joint state
        self.publisher.publish(joint_state)



def main(args=None):
    rclpy.init(args=args)
    node = WheelAndSensorJointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

