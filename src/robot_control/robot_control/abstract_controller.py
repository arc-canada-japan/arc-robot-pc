from abc import ABC, abstractmethod
import array
import rclpy
from rclpy.node import Node
from typing import List
from std_msgs.msg import String, Bool, Float32MultiArray
from enum import Enum

class Arm(Enum):
    """
        Enum class to define the arm side. It corresponds to the arm_id in the joint values received by the robot.
    """
    NONE = 0
    LEFT = 1
    RIGHT = 2

class AbstractController(ABC, Node):
    """
    Abstract class for the robot controller. It defines the common methods and attributes.
    All the controllers must inherit from this class. For basic functionnalities, the definition
    from this abstract class can be used. For specific functionnalities, the methods can be overriden.
    """
    # CONSTANTS --------------------------------------------------------------
    # Those attributes shouldn't be modified by the child classes. They are defined in the constructor.
    ROBOT_NAME: str # Name of the robot
    JOINTS_NUMBER: int # Number of joints of the robot
    ARM_NUMBER: int # Number of arms of the robot (1 or 2)

    # The initial position of the robot (when the application is launched). The setter checks the type and the length of the list.
    @property
    def INIT_POS(self) -> List[float]:
        return self._init_pos
    
    @INIT_POS.setter
    def INIT_POS(self, init_pos):
        # Check if init_pos is an array of type 'd' and convert to list
        if isinstance(init_pos, array.array) and init_pos.typecode == 'd':
            init_pos = list(init_pos)
        # Check if init_pos is a list of floats
        if not isinstance(init_pos, list) or not all(isinstance(i, float) for i in init_pos):
            raise TypeError(f"INIT_POS must be a list/array of floats, got {init_pos} of type {type(init_pos)}")
        # Check if the length of init_pos is correct
        if len(init_pos) != self.JOINTS_NUMBER:
            raise ValueError(f"INIT_POS must have {self.JOINTS_NUMBER} elements")
        # Set the _init_pos attribute
        self._init_pos = init_pos

    # The home position of the robot (as defined by the robot documentation). The setter checks the type and the length of the list.
    @property
    def HOME_POS(self) -> List[float]:
        return self._home_pos
    
    @HOME_POS.setter
    def HOME_POS(self, home_pos):
        # Check if home_pos is an array of type 'd' and convert to list
        if isinstance(home_pos, array.array) and home_pos.typecode == 'd':
            home_pos = list(home_pos)
        # Check if home_pos is a list of floats
        if not isinstance(home_pos, list) or not all(isinstance(i, float) for i in home_pos):
            raise TypeError(f"HOME_POS must be a list/array of floats, got {home_pos} of type {type(home_pos)}")        
        # Check if the length of home_pos is correct
        if len(home_pos) != self.JOINTS_NUMBER:
            raise ValueError(f"HOME_POS must have {self.JOINTS_NUMBER} elements")
        # Set the _home_pos attribute
        self._home_pos = home_pos

    # METHODS ----------------------------------------------------------------

    def __init__(self, robot_name) -> None:
        """
            Constructor of the AbstractController class. It initializes the common attributes and calls the Node constructor.
            The robot_name is the name of the robot. It is used to create the node name (by adding "_controller" as suffix).
            The attributes JOINTS_NUMBER and ARM_NUMBER are declared as parameters in the ROS node. Then, they should be 
            defined in the config file of the robot.

            :param robot_name: (str) the name of the robot.
        """
        self.ROBOT_NAME = robot_name
        #Node.__init__(self, node_name=self.ROBOT_NAME+'_controller')
        super().__init__(node_name=self.ROBOT_NAME+'_controller')
        self.get_logger().info("========= "+self.ROBOT_NAME+" CONTROLLER =========")   
        self._init_pos = None
        
        self.declare_parameter('robot_joints_number', 0)
        self.JOINTS_NUMBER = self.get_parameter('robot_joints_number').get_parameter_value().integer_value

        self._home_pos = [0.0] * self.JOINTS_NUMBER # Default value

        self.declare_parameter('robot_arm_number', 1)
        self.ARM_NUMBER = self.get_parameter('robot_arm_number').get_parameter_value().integer_value

        self.create_ros_subscribers()
        self.create_ros_publishers()

    # ROS Initialisation methods
    @abstractmethod
    def declare_ros_parameters(self) -> None:
        """
            Declare the ROS parameters needed by the robot controller, and that are specific.
            It should be overriden by the child classes.
        """
        pass

    def create_ros_subscribers(self):
        """
            Create the ROS subscribers needed by the robot controller. It creates the minimal subscriptions, including:
             * the joint values, 
             * the emergency stop 
             * and the joint values as string.

            It can be overriden by the child classes to add more subscriptions.
        """
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
        """
            Create the ROS publishers needed by the robot controller. It creates the minimal publishers, including:
             * the joint values to send to the Operator PC.

            It can be overriden by the child classes to add more publishers.
        """
        # Create the publisher to send the joint values to the Operator PC
        self.robot_joints_val_pub = self.create_publisher(Float32MultiArray, "robot_joint_values", 10)

    # ROS Callback methods
    @abstractmethod
    def robot_joints_callback(self, cmd) -> None:
        """
            Callback method for the joint values. It should be overriden by the child classes.
            This method is called when the robot receives new joint values. It should compute the real robot joint values (if needed).
            Then it should call the move_robot method to move the robot to the new joint values.

            :param cmd: (Float32MultiArray) the new joint values received by the robot. The joint values are received as a list of 6 float values, 
                        representing the joint angles in degree. The first value in the list is the arm_id, then the joint values. Hence, the list 
                        length is equal to the number of joints + 1.
        """
        pass

    @abstractmethod
    def emergency_stop_callback(self, msg) -> None:
        """
            Callback method for the emergency stop. It should be overriden by the child classes.
            This method is called when the robot receives an emergency stop signal. It should stop the robot.
            If the robot API has a specific emergency method to stop the robot, it should be called here.
            Otherwise, the robot should be stopped with another method.
            During the emergency stop, the robot should not move anymore (even if a joint_value is received).
            

            :param msg: (Bool) the emergency stop message. If the message is True, the robot should stop.
        """
        pass

    def joint_print_callback(self, msg) -> None:
        """
            Callback method for the joint values as string. It prints the received joint values as string.
            This method can be used to debug the robot controller.

            :param msg: (String) the joint values as string.
        """
        self.get_logger().info('"%s"' % msg.data)

    # Robot control methods
    @abstractmethod
    def move_robot(self, joint_values, arm_id=1) -> None:
        """
            Move the robot to the given joint values. It should be overriden by the child classes.

            :param joint_values: (List[float]) the joint values to move the robot to.
            :param arm_id: (int) the arm id to move. Default is 1.
        """
        pass

    def move_to_home_position(self) -> None:
        """
            Move the robot to the home position. It calls the move_robot method with the home position.
        """
        self.move_robot(self.HOME_POS)

    def move_to_init_position(self) -> None:
        """
            Move the robot to the initial position. It calls the move_robot method with the initial position.
        """
        self.move_robot(self.INIT_POS)