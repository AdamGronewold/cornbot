import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import QuaternionStamped
import matplotlib.pyplot as plt

class FeelerStatePlotter(Node):
    def __init__(self):
        super().__init__('feeler_state_plotter')

        # Subscribers to ROS2 topics
        self.sub_left_state = self.create_subscription(KeyValue, 'cornbot/feeler/left_contact_state', self.left_state_callback, 10)
        self.sub_left_angle = self.create_subscription(QuaternionStamped, 'cornbot/feeler/left_state', self.left_angle_callback, 10)

        # Initialize plot data storage
        self.left_angle = []
        self.left_time = []
        self.left_contact_state = []
        self.start_time = None

        # Define the state mapping
        self.state_map = {
            'Unknown State': -2,
            'Steady State Angle': 0,
            'In Contact': 9,
            'SHM Increasing': 3,
            'SHM Decreasing': 4,
            'Nearing Crash': 10
        }

        # Set up the plots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.ax1.set_ylabel('Angle (degrees)')
        self.ax2.set_ylabel('Contact State')
        self.ax2.set_yticks(list(self.state_map.values()))
        self.ax2.set_yticklabels(list(self.state_map.keys()))
        self.ax2.set_xlabel('Time (s)')

    def left_state_callback(self, msg):
        self.left_contact_state.append(self.state_map.get(msg.key, 0))  # Default to 'Unknown State'
        #print(self.left_contact_state[-1])

    def left_angle_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.left_time.append(current_time - self.start_time)
        self.left_angle.append(msg.quaternion.x)
        self.update_plot()

    def update_plot(self):
        # Clear the plots
        self.ax1.clear()
        self.ax2.clear()

        # Plot the angle
        self.ax1.plot(self.left_time, self.left_angle, color='blue')
        self.ax1.set_ylabel('Angle (degrees)')

        # Plot the contact state
        self.ax2.plot(self.left_time, self.left_contact_state, color='red')
        self.ax2.set_yticks(list(self.state_map.values()))
        self.ax2.set_yticklabels(list(self.state_map.keys()))
        self.ax2.set_ylabel('Contact State')
        self.ax2.set_xlabel('Time (s)')

        # Refresh the plot
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = FeelerStatePlotter()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

