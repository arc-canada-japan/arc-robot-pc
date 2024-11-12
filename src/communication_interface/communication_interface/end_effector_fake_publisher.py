import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from ament_index_python.packages import get_package_share_directory
import os

class EndEffectorPublisher(Node):
    def __init__(self):
        super().__init__('end_effector_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'end_effector_position', 10)
        self.declare_parameter('fake_data', '')
        self.file_path = self.get_parameter('fake_data').get_parameter_value().string_value
        self.load_data()
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz
        self.data_index = 0

    def load_data(self):
        # Read file and store each line as a list of floats
        with open(self.file_path, 'r') as file:
            self.data = []
            for line in file:
                values = [float(x) for x in line.strip().split(',')]
                self.data.append(values)
        self.get_logger().info(f"Loaded {len(self.data)} data points from {self.file_path}")

    def publish_data(self):
        # Loop back to the start if we have reached the end of the data
        if self.data_index >= len(self.data):
            self.data_index = 0
            self.get_logger().info("Restarting data from the beginning.")

        # Create Float32MultiArray message
        msg = Float32MultiArray()
        msg.data = self.data[self.data_index]

        # Publish message and log
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published data point {self.data_index + 1}: {msg.data}")

        # Move to the next data point
        self.data_index += 1

def main(args=None):
    rclpy.init(args=args)

    end_effector_publisher = EndEffectorPublisher()

    try:
        rclpy.spin(end_effector_publisher)
    except KeyboardInterrupt:
        pass

    # Clean up on shutdown
    end_effector_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
