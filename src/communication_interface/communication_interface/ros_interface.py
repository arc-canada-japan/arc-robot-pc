import array
from communication_interface.abstract_interface import AbstractInterface
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, Int64, Int32MultiArray
import enum
from functools import partial
import threading
from rclpy.qos import QoSProfile

class PubData(enum.Enum):
    PUB = 0
    TYPE = 1

class RosInterface(AbstractInterface):
    TypeBoolAlias = Bool
    TypeIntAlias = Int64
    TypeStrAlias = String
    TypeFloatArrayAlias = Float32MultiArray
    TypeIntArrayAlias = Int32MultiArray

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
            # Use partial to create a callback function with the topic pre-set
            callback_with_topic = partial(self._common_callback, topic=topic)
            self._subscriber_list[topic] = (callback,
                                            self._node.create_subscription(
                                            type,
                                            topic,
                                            callback_with_topic,
                                            10)
            )
        self._node.get_logger().info(f"Subscribers created: {self._subscriber_list}")
            
    def define_publishers(self, pub_list: dict) -> None:
        for topic, type in pub_list.items():
            self._publisher_list[topic] = (self._node.create_publisher(type, topic, 10), type)
        self._node.get_logger().info(f"Publishers created: {self._publisher_list}")


    def publish(self, pub_topic: str, data) -> None:
        try:
            if not isinstance(data, self._publisher_list[pub_topic][PubData.TYPE.value]):
                new_data = self._publisher_list[pub_topic][PubData.TYPE.value]()
                new_data.data = data
                data = new_data
            self._publisher_list[pub_topic][PubData.PUB.value].publish(data)
        except TypeError as e:
            self._node.get_logger().error(f"Error while publishing the data, the given type is wrong: {e}")

    def modify_qos_for_topic(self, topic: str, new_qos: QoSProfile, new_callback=None) -> None:
        """
        Recreate the subscriber for the given topic with a new QoS profile.

        :param topic: The topic to modify.
        :param new_qos: The new QoSProfile.
        :param new_callback: Optional new callback function. If None, the existing one will be used.
        """
        if topic not in self._subscriber_list:
            self._node.get_logger().error(f"No subscriber found for topic '{topic}'")
            return

        # Get the old callback and type
        old_callback, old_subscription = self._subscriber_list[topic]
        sub_type = old_subscription.msg_type

        # Destroy the old subscription
        self._node.destroy_subscription(old_subscription)

        # If no new callback is provided, use the existing one
        if new_callback is None:
            new_callback = old_callback

        # Create a new callback with partial (same as in define_subscribers)
        callback_with_topic = partial(self._common_callback, topic=topic)
        
        # Create the new subscription
        new_subscription = self._node.create_subscription(
            sub_type,
            topic,
            callback_with_topic,
            new_qos
        )

    def _common_callback(self, msg, topic):
        """
            Common callback function for all the subscribers. It calls the callback function of the subscriber in a thread.
            It is used to call the callback and sending it directly the data (avoing to use msg.data). This is done for usability reasons.
            All the callbacks of the controllers are expecting to receive the data directly, not the message.
            The thread is used to avoid blocking the current callback.

            :param msg: the message received
            :param topic: the topic where the message was received
        """
        th = threading.Thread(target=self._subscriber_list[topic][0],
                              args=(msg.data,), 
                              daemon=True)
        th.start() # call the callback in a thread to avoid blocking the main thread