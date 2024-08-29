import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import QuaternionStamped
import numpy as np

class FeelerStateIdentificationNode(Node):
    def __init__(self):
        super().__init__('feeler_state_identification')
        self.subscription_left = self.create_subscription(QuaternionStamped, 'feeler/left_state', self.left_callback, 3)
        self.subscription_right = self.create_subscription(QuaternionStamped, 'feeler/right_state', self.right_callback, 3)
        
        # Publishers for left and right states
        self.publisher_left_state = self.create_publisher(KeyValue, 'feeler/left_contact_state', 3)
        self.publisher_right_state = self.create_publisher(KeyValue, 'feeler/right_contact_state', 3)
        
        # Numerical state publishers
        self.publisher_left_state_numerical = self.create_publisher(QuaternionStamped, 'feeler/left_contact_state_numerical', 3)
        self.publisher_right_state_numerical = self.create_publisher(QuaternionStamped, 'feeler/right_contact_state_numerical', 3)
        
        self.classifier_window = 5
        self.left_state = "Unknown State"
        self.right_state = "Unknown State"

        # Histories for left and right sensors
        self.left_angle_history = []
        self.left_vel_history = []
        self.left_acc_history = []
        self.left_time_history = []

        self.right_angle_history = []
        self.right_vel_history = []
        self.right_acc_history = []
        self.right_time_history = []

        # Assume mounting angle is known for each sensor
        self.left_mounting_angle = np.deg2rad(90)  # For example
        self.right_mounting_angle = 0

        # State mapping to numerical values
        self.state_to_number = {
            "Unknown": -20.0,
            "Steady State Angle": 0.0,
            "SHM Increasing": 50.0,
            "SHM Decreasing": 25.0,
            "In Contact": 80.0,
            "Nearing Crash": 90.0
        }

    def left_callback(self, msg):
        self.id_callback("left", msg)

    def right_callback(self, msg):
        self.id_callback("right", msg)

    def id_callback(self, side, msg):
        angle = msg.quaternion.x
        velocity = msg.quaternion.y
        acceleration = msg.quaternion.z
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        if side == "left":
            if self.left_mounting_angle == np.deg2rad(90):
                angle = -angle
                velocity = -velocity
                acceleration = -acceleration
            self.update_history(self.left_angle_history, angle, self.left_time_history, timestamp)
            self.update_history(self.left_vel_history, velocity)
            self.update_history(self.left_acc_history, acceleration)
            state = self.classify(self.left_angle_history, self.left_vel_history, self.left_acc_history, self.left_state)
            self.left_state = state
            self.publish_state("left", state, msg.header.stamp)
        else:
            if self.right_mounting_angle == np.deg2rad(90):
                angle = -angle
                velocity = -velocity
                acceleration = -acceleration
            self.update_history(self.right_angle_history, angle, self.right_time_history, timestamp)
            self.update_history(self.right_vel_history, velocity)
            self.update_history(self.right_acc_history, acceleration)
            state = self.classify(self.right_angle_history, self.right_vel_history, self.right_acc_history, self.right_state)
            self.right_state = state
            self.publish_state("right", state, msg.header.stamp)

    def update_history(self, history, value, time_history=None, timestamp=None):
        history.append(value)
        if time_history is not None:
            time_history.append(timestamp)
            if len(history) != len(time_history):
                min_len = min(len(history), len(time_history))
                history = history[:min_len]
                time_history = time_history[:min_len]
        
        if len(history) > self.classifier_window + 100:
            history.pop(0)
            if time_history is not None:
                time_history.pop(0)

    def classify(self, angle_history, velocity_history, acceleration_history, current_state):
        i = len(angle_history)-1
        if i <= self.classifier_window:
            return "Unknown State"

        y_angle_h = np.array(angle_history)
        ydot_angle_h = np.array(velocity_history)
        yddot_angle_h = np.array(acceleration_history)

        if current_state == "Unknown State":
            if all(np.abs(ydot_angle_h[i - self.classifier_window:i]) < 40):
                return "Steady State Angle"
            elif ydot_angle_h[i] < 0 and yddot_angle_h[i] < -10000:
                return "In Contact"
            elif all(ydot_angle_h[i - self.classifier_window:i] > 0):
                return "SHM Increasing"
            elif all(ydot_angle_h[i - self.classifier_window:i] < 0) and all(y_angle_h[i - self.classifier_window:i] > 0):
                return "SHM Decreasing"

        elif current_state == "Steady State Angle":
            if ydot_angle_h[i] < -20 and yddot_angle_h[i] < -500:
                return "In Contact"

        elif current_state == "SHM Increasing":
            if all(np.abs(ydot_angle_h[i - self.classifier_window:i]) < 20):
                return "Steady State Angle"
            elif y_angle_h[i] > 0 and np.abs(ydot_angle_h[i]) < 300 and yddot_angle_h[i] < 0:
                return "SHM Decreasing"
            elif ydot_angle_h[i] < -30 and yddot_angle_h[i] > 0:
                return "In Contact"

        elif current_state == "SHM Decreasing":
            if y_angle_h[i] < 0 and np.abs(ydot_angle_h[i]) < 15 and yddot_angle_h[i] > 0:
                return "SHM Increasing"
            elif all(np.abs(ydot_angle_h[i - self.classifier_window:i]) < 5):
                return "Steady State Angle"
            elif ydot_angle_h[i] < 0 and yddot_angle_h[i] < -5000:
                return "In Contact"


        elif current_state == "In Contact":
            if all(ydot_angle_h[i - self.classifier_window:i] > 0):
                return "SHM Increasing"
            elif all(abs(ydot_angle_h[i - self.classifier_window:i] < 0.45)) and  all(abs(yddot_angle_h[i - self.classifier_window:i]) < 90):
            	return "Steady State Angle"
            elif y_angle_h[i - 1] < -75:
                return "Nearing Crash"

        elif current_state == "Nearing Crash":
            if all(ydot_angle_h[i - self.classifier_window:i] > 0):
                return "SHM Increasing"
        
        return current_state

    def publish_state(self, side, state, timestamp):
        msg = KeyValue()
        msg.key = state
        msg.value = f"{timestamp.sec}.{int(timestamp.nanosec/1e6)}"

        # Publish the string state
        if side == "left":
            self.publisher_left_state.publish(msg)
        else:
            self.publisher_right_state.publish(msg)

        # Publish the numerical state as QuaternionStamped
        num_msg = QuaternionStamped()
        num_msg.header.stamp = timestamp
 
        num_msg.quaternion.x = float(self.state_to_number.get(state, -1))  # Default to -1 for unknown states
        num_msg.quaternion.y = 0.0
        num_msg.quaternion.z = 0.0
        num_msg.quaternion.w = 0.0

        if side == "left":
            num_msg.header.frame_id='feeler_left_frame'
            self.publisher_left_state_numerical.publish(num_msg)
        else:
            num_msg.header.frame_id='feeler_right_frame'
            self.publisher_right_state_numerical.publish(num_msg)

def main(args=None):
    rclpy.init(args=args)

    node = FeelerStateIdentificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

