import zmq
import yaml
from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node
import enum
import threading

# Enumeration for the type of the data and callback (for readability)
class SubData(enum.Enum):
    CALLBACK = 0
    TYPE = 1

class ZmqInterface(AbstractInterface):
    def __init__(self, node: Node):
        super().__init__(interface_name="ZMQ", # The name of the interface
                         node=node # The ROS node
                        )
        
        # Calling the listening function in a thread
        self.listening = True
        self._th = threading.Thread(target=self.listening_thread)
        
    def __del__(self):
        self.listening = False
        self._subscriber.close()
        self._publisher.close()
        self._context.term()

    def load_parameters(self) -> None:
        with open(self._config_file, 'r') as f:
            params = yaml.safe_load(f)
            self._parameters = params['/**']['ros__parameters']

            self.ip = self._parameters['tcp_ip']
            self.port_sub = self._parameters['tcp_port']
            self.port_pub = self._parameters['tcp_port'] + 1
     
    def connection_to_host(self) -> None:
        self._context = zmq.Context()

    def define_subscribers(self, sub_list: dict) -> None:
        if not sub_list or sub_list == {}:
            return        
        self._subscriber_list = sub_list

        self._subscriber = self._context.socket(zmq.SUB)
        self._subscriber.connect(f"tcp://{self.ip}:{self.port_sub}")

        txt_topics = ""
        for topic in self._subscriber_list:
            self._subscriber.subscribe(topic)
            txt_topics += f"{topic}, "

        self._poller = zmq.Poller()
        self._poller.register(self._subscriber, zmq.POLLIN)

        self._node.get_logger().info(f"Subscriber created (tcp://{self.ip}:{self.port_sub})")
        self._node.get_logger().info(f"Subscribed to the following topics: {txt_topics}")       
        self._th.start()

    def define_publishers(self, pub_list: dict) -> None:
        if not pub_list or pub_list == {}:
            return
        self._publisher_list = pub_list
        self._publisher = self._context.socket(zmq.PUB)
        self._publisher.connect(f"tcp://{self.ip}:{self.port_pub}")
        self._node.get_logger().info(f"Publisher created (tcp://{self.ip}:{self.port_pub})")

    def publish(self, pub_topic: str, data) -> None:
         # check if the data type is good
        if not isinstance(data, self._publisher_list[pub_topic]):
            try:
                data = self._publisher_list[pub_topic](data)
            except Exception as e:
                self._node.get_logger().error(f"Error while publishing the data, the given type is wrong and the conversion failed: {e}")
                return
        self._publisher.send_string(pub_topic, flags=zmq.SNDMORE) # send the topic first
        self._publisher.send_json(data) # then, send the data

    def listening_thread(self):
        while self.listening:
            evts = dict(self._poller.poll(timeout=100))
            if self._subscriber in evts:
                try:
                    topic = self._subscriber.recv_string()
                    data = self._subscriber.recv_json()                    
                    if not isinstance(data, self._subscriber_list[topic][SubData.TYPE.value]):
                        data = self._subscriber_list[topic][SubData.TYPE.value](data)
                    self._subscriber_list[topic][SubData.CALLBACK.value](data) # call the callback
                except Exception as e:
                    self._node.get_logger().error(f"Error while receiving data: {e}")
                    continue

