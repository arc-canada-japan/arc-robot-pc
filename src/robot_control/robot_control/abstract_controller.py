from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from typing import List
from std_msgs.msg import String, Bool, Float32MultiArray

class AbstractController(ABC, Node):
    ROBOT_NAME: str
    JOINTS_NUMBER: int

    @property
    def INIT_POS(self) -> List[float]:
        return self._init_pos
    
    @INIT_POS.setter
    def INIT_POS(self, init_pos):
        if not isinstance(init_pos, list) or not all(isinstance(i, float) for i in init_pos):
            raise TypeError("INIT_POS must be a list of floats")
        if len(init_pos) != self.JOINTS_NUMBER:
            raise ValueError(f"INIT_POS must have {self.JOINTS_NUMBER} elements")
        self._init_pos = init_pos

    # METHODS ----------------------------------------------------------------

    def __init__(self, robot_name) -> None:
        self.ROBOT_NAME = robot_name
        self._init_pos = None
        Node.__init__(self, node_name=self.ROBOT_NAME+'_controller')

        self.declare_parameter('robot_joints_number', 0)
        self.JOINTS_NUMBER = self.get_parameter('robot_joints_number').get_parameter_value().integer_value

        self.declare_parameter('robot_arm_number', 1)
        self.ARM_NUMBER = self.get_parameter('robot_arm_number').get_parameter_value().integer_value

        self.create_ros_subscribers()
        self.create_ros_publishers()

        self.get_logger().info("========= "+self.ROBOT_NAME+" CONTROLLER =========")   

    # ROS Initialisation methods
    @abstractmethod
    def declare_ros_parameters(self) -> None:
        """
            Declare the ROS parameters needed by the robot controller, and thatare specific.
        """
        pass

    def create_ros_subscribers(self):
        # Create the subscriptions to the joint values, emergency stop and joint values as string
        self.joints_val_str = self.create_subscription(
            String,
            'joint_value_str',
            self.joint_print_callback,
            10)
        self.joints_val_str  # prevent unused variable warning

        self.emergency = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10)
        self.emergency  # prevent unused variable warning

        if self.ARM_NUMBER == 1:
            self.joints_val = self.create_subscription(
                Float32MultiArray,
                'joint_value',
                self.robot_joints_callback,
                10)
            self.joints_val  # prevent unused variable warning
        else:
            self.joints_val_sub = []
            for i in range(self.ARM_NUMBER):
                self.joints_val_sub.append(self.create_subscription(
                    Float32MultiArray, # TO CHECK
                    'joint_value_'+str(i),
                    self.robot_joints_callback,
                    10)
                )

    def create_ros_publishers(self) -> None:
        # Create the publisher to send the joint values to the Operator PC
        self.robot_joints_val_pub = self.create_publisher(Float32MultiArray, "robot_joint_values", 10)

    # ROS Callback methods
    @abstractmethod
    def robot_joints_callback(self, cmd) -> None:
        pass

    @abstractmethod
    def emergency_stop_callback(self, msg) -> None:
        pass

    def joint_print_callback(self, msg) -> None:
        self.get_logger().info('"%s"' % msg.data)

    # Robot control methods
    @abstractmethod
    def move_robot(self, joint_values) -> None:
        pass

    def move_to_home_position(self) -> None:
        """
        Move the robot to the home position.
        """
        self.move_robot(self.HOME_POS)

    def move_to_init_position(self) -> None:
        """
        Move the robot to the initial position.
        """
        self.move_robot(self.INIT_POS)