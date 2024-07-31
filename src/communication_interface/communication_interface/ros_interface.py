from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node

class RosInterface(AbstractInterface, Node):
    def __init__(self, node_name: str):
        super().__init__(interface_name="ROS", node_name=node_name)

    def load_parameters(self) -> None:
        pass
     

    def connection_to_host(self, host: str, port: int) -> None:
        pass


    def define_subscribers(self, sub_list: dict) -> None:
        for topic, (callback, type) in sub_list.items():
            self._subscriber_list[topic] = self.create_subscription(
            type,
            topic,
            callback,
            10)
            
    def define_publishers(self, pub_list: dict) -> None:
        for topic, type in pub_list.items():
            self._publisher_list[topic] = self.create_publisher(type, topic, 10)


    def publish(self, pub_topic: str, data) -> None:
        try:
            self._publisher_list[pub_topic].publish(data)
        except TypeError as e:
            self.get_logger().error(f"Error while publishing the data, the given type is wrong: {e}")
