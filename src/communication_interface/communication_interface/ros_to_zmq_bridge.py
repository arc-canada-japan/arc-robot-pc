import threading
import yaml
import zmq
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from functools import partial
import os

class ZmqRosInterface(Node):
    def __init__(self) -> None:
        super().__init__('ROS_to_ZMQ_Interface')

        # Get the configuration file path from the ROS parameter
        self.declare_parameter('config_file', '')
        self._config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self._parameters = None

        if not self._config_file or not os.path.isfile(self._config_file):
            self.get_logger().error(f"Invalid or missing configuration file: {self._config_file}")
            return

        self.ZMQ_load_parameters()
        self.ZMQ_context = zmq.Context()
        self.socket_pub = self.ZMQ_context.socket(zmq.PUB)
        self.socket_pub.connect(self.url_pub)

        self.socket_sub = self.ZMQ_context.socket(zmq.SUB)
        self.socket_sub.connect(self.url_sub)
        self.socket_sub.subscribe('robot_joint_values')

        self._poller = zmq.Poller()
        self._poller.register(self.socket_sub, zmq.POLLIN)

        self.listening = True
        self._th = threading.Thread(target=self.ZMQ_listening_thread, daemon=True)
        self._th.start()

        self.create_subscription(
            Float32MultiArray,
            'end_effector_position',
            partial(self.ZMQ_publish, 'end_effector_position'),
            10
        )

        self.ROS_pub = self.create_publisher(Float32MultiArray, 'robot_joint_values', 10)

        self.get_logger().info("ROS to ZMQ Interface has been initialized")

    def ZMQ_publish(self, topic: str, data: dict) -> None:
        self.socket_pub.send_string(topic, flags=zmq.SNDMORE)
        self.socket_pub.send_string(json.dumps(list(data.data)))
        self.get_logger().info(f"ZMQ: Published data to topic {topic}: {data}")

    def ZMQ_load_parameters(self) -> None:
        with open(self._config_file, 'r') as f:
            params = yaml.safe_load(f)
            self._parameters = params['/**']['ros__parameters']

            self.ip = self._parameters['host_ip']
            self.port = self._parameters['host_port']
            self.protocol = self._parameters['protocol']
            self.ws_path_pub = self._parameters['ws_path_pub']
            self.ws_path_sub = self._parameters['ws_path_sub']

            self.url_pub = f"{self.protocol}://{self.ip}:{self.port}{self.ws_path_pub if self.protocol == 'ws' else ''}"
            self.url_sub = f"{self.protocol}://{self.ip}:{self.port}{self.ws_path_sub if self.protocol == 'ws' else ''}"

            self.get_logger().info(f"URL for publishing: {self.url_pub}")
            self.get_logger().info(f"URL for subscribing: {self.url_sub}")

    def ZMQ_listening_thread(self):
        while self.listening:
            evts = dict(self._poller.poll(timeout=100))
            if self.socket_sub in evts:
                try:
                    topic = self.socket_sub.recv_string()
                    data = self.socket_sub.recv_string()

                    self.get_logger().info(f"ZMQ: Received data from topic {topic}: {data}")
                    data = json.loads(data)

                    if not isinstance(data, list):
                        data = list(data)

                    msg = Float32MultiArray()
                    msg.data = data
                    self.ROS_pub.publish(msg)
                except Exception as e:
                    self.get_logger().error(f"Error while receiving data: {e}")
                    continue


def main(args=None):
    rclpy.init(args=args)
    node = ZmqRosInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
