from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node

class RosInterface(AbstractInterface):
    def __init__(self, node: Node):
        super().__init__(interface_name="ROS", # The name of the interface
                         node=node # The ROS node
                        )

    def load_parameters(self) -> None:
        pass
     
    def connection_to_host(self) -> None:
        pass


    def define_subscribers(self, sub_list: dict) -> None:
        for topic, (callback, type) in sub_list.items():
            self._subscriber_list[topic] = self._node.create_subscription(
            type,
            topic,
            callback,
            10)
            
    def define_publishers(self, pub_list: dict) -> None:
        for topic, type in pub_list.items():
            self._publisher_list[topic] = self._node.create_publisher(type, topic, 10)


    def publish(self, pub_topic: str, data) -> None:
        try:
            self._publisher_list[pub_topic].publish(data)
        except TypeError as e:
            self._node.get_logger().error(f"Error while publishing the data, the given type is wrong: {e}")
