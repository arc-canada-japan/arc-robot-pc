import zmq
import yaml
from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node

class NepInterface(AbstractInterface):
    def __init__(self, node: Node):
        super().__init__(interface_name="ZMQ", # The name of the interface
                         node=node # The ROS node
                        )

    def load_parameters(self) -> None:
        with open(self._config_file, 'r') as f:
            params = yaml.safe_load(f)
            self._parameters = params['/**']['ros__parameters']

            self.ip = self._parameters['tcp_ip']
            self.port = self._parameters['tcp_port']
     
    def connection_to_host(self) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{self.ip}:{self.port}")

    def define_subscribers(self, sub_list: dict) -> None:
        pass
            
    def define_publishers(self, pub_list: dict) -> None:
        pass


    def publish(self, pub_topic: str, data) -> None:
        pass

