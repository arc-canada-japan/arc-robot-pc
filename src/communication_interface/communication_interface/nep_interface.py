import nep
from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node

### TODO

class NepInterface(AbstractInterface):
    def __init__(self, node: Node):
        super().__init__(interface_name="NEP", # The name of the interface
                         node=node # The ROS node
                        )

    def load_parameters(self) -> None:
        pass
     
    def connection_to_host(self) -> None:
        pass


    def define_subscribers(self, sub_list: dict) -> None:
        pass
            
    def define_publishers(self, pub_list: dict) -> None:
        pass


    def publish(self, pub_topic: str, data) -> None:
        pass

