import threading
import yaml
import zmq
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from rclpy_message_converter import json_message_converter
from functools import partial
import os
import importlib

class RosToZmqInterface(Node):
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
        for topic in self.output_topic_message_map.keys():
            self.socket_sub.subscribe(topic)
        #self.socket_sub.subscribe('robot_joint_values')

        self._poller = zmq.Poller()
        self._poller.register(self.socket_sub, zmq.POLLIN)

        self.listening = True
        self._th = threading.Thread(target=self.ZMQ_listening_thread, daemon=True)
        self._th.start()

        self.define_ros_subscribers()
        self.define_ros_publishers()

        # self.create_subscription(
        #     Float32MultiArray,
        #     'end_effector_position',
        #     partial(self.ZMQ_publish, 'end_effector_position'),
        #     10
        # )

        # self.ROS_pub = self.create_publisher(Float32MultiArray, 'robot_joint_values', 10)

        self.get_logger().info("ROS to ZMQ Interface has been initialized")

    def ZMQ_publish(self, topic: str, data) -> None:
        # Convert ROS message to JSON-compatible format
        json_data = json_message_converter.convert_ros_message_to_json(data)
        zmq_data = topic+"?"+json_data
        # self.socket_pub.send_string(topic, flags=zmq.SNDMORE)
        self.socket_pub.send_string(zmq_data)
        self.get_logger().info(f"ZMQ: Published data to topic {topic}: {json_data}")

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

            # Load ros_input and ros_output topics dynamically
            ros_input = params['/**'].get('ros_input', {})
            ros_output = params['/**'].get('ros_output', {})

            # Helper function to dynamically import message types
            def get_message_type(msg_type_str):
                module_name, class_name = msg_type_str.rsplit('/', 1)
                module = importlib.import_module(f"{module_name}.msg")
                return getattr(module, class_name)

            self.input_topic_message_map = {}
            self.output_topic_message_map = {}
            
            # Populate the input dictionary with topics and their respective message types
            for topic, msg_type in ros_input.items():
                self.input_topic_message_map[topic] = get_message_type(msg_type)

            # Populate the output dictionary with topics and their respective message types
            for topic, msg_type in ros_output.items():
                self.output_topic_message_map[topic] = get_message_type(msg_type)

            self.get_logger().info(f"Input Topic-Message Map: {self.input_topic_message_map}")
            self.get_logger().info(f"Output Topic-Message Map: {self.output_topic_message_map}")

    def define_ros_subscribers(self) -> None:
        for topic, msg_type in self.input_topic_message_map.items():
            self.create_subscription(
                msg_type,
                topic,
                partial(self.ZMQ_publish, topic),
                10
            )

    def define_ros_publishers(self) -> None:
        self.ROS_pub = {}
        for topic, msg_type in self.output_topic_message_map.items():
            self.ROS_pub[topic] = self.create_publisher(msg_type, topic, 10)

    def ZMQ_listening_thread(self):
        while self.listening:
            evts = dict(self._poller.poll(timeout=100))
            if self.socket_sub in evts:
                try:
                    # topic = self.socket_sub.recv_string()
                    full_data = self.socket_sub.recv_string()

                    index = full_data.find("?")

                    topic = full_data[:index]
                    data = full_data[(index+1):]

                    self.get_logger().info(f"ZMQ: Received data from topic {topic}: {data}")
                    data = json.loads(data)

                    if topic in self.output_topic_message_map:
                        msg_type = self.output_topic_message_map[topic]
                        
                        msg = json_message_converter.convert_json_to_ros_message(msg_type, data)

                        self.ROS_pub[topic].publish(msg)
                    else:
                        self.get_logger().error(f"The topic '{topic}' not in output map")
                except Exception as e:
                    self.get_logger().error(f"Error while receiving data: {e}")
                    continue


def main(args=None):
    rclpy.init(args=args)
    node = RosToZmqInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
