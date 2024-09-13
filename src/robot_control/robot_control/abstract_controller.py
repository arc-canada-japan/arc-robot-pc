from abc import ABC, abstractmethod
import array
from rclpy.node import Node
from typing import List
from std_msgs.msg import String, Bool, Float32MultiArray
from enum import Enum
from communication_interface import * # Import all classes from the communication_interface package (no need to use the module name)

class ArmLeg(Enum):
    """
        Enum class to define the arm leg side. For code readability, the arm/leg side can be defined with this enum.
    """
    ARM = 0
    LEG = 1
    WHEEL = 2

class ArmLegSide(Enum):
    """
        Enum class to define the arm/leg side. It corresponds to the arm_id/leg_id in the joint values received by the robot.
    """
    NONE = 0
    LEFT = 1
    RIGHT = 2

class ArticulationDesc:
    """
        Descriptor class for the articulation attributes of the robot controller. It checks the type and the length of the list.
    """
    def __set_name__(self, owner, name):
        self.name = name

    def __get__(self, instance, owner):
        return instance.__dict__[self.name]# or [0.0] * instance.ARM_JOINTS_NUMBER
    
    def __set__(self, instance, value):
        if isinstance(value, array.array) and value.typecode == 'd':
            instance.__dict__[self.name] = list(value)
            return
        if not isinstance(value, list) or not all(isinstance(i, float) for i in value):
            raise TypeError(f"{self.name} must be a list/array of floats, got {value} of type {type(value)}")
        if len(value) != instance.LEG_JOINTS_NUMBER or len(value) != instance.ARM_JOINTS_NUMBER: # TODO: adapt automatically to the number of joints depending on the limb
            raise ValueError(f"{self.name} must have {instance.LEG_JOINTS_NUMBER} or {instance.ARM_JOINTS_NUMBER} elements")
        instance.__dict__[self.name] = value

    def __str__(self) -> str:
        return str(self.__get__(self, self.__class__))

class AbstractController(ABC, Node):
    """
        Abstract class for the robot controller. It defines the common methods and attributes.
        All the controllers must inherit from this class. For basic functionnalities, the definition
        from this abstract class can be used. For specific functionnalities, the methods can be overriden.

        The communication is done via the communication_interface ROS package. The interface used is defined
        by the launch argument 'communication_interface'. It allows the package to be versatile, and use different
        communication interfaces (ROS, ZMQ, etc.) without changing the code.
    """
    # READ-ONLY ATTRIBUTES ---------------------------------------------------
    @property # Name of the robot
    def ROBOT_NAME(self) -> str:
        return self._ROBOT_NAME
    
    @property # Number of joints of the arm of robot
    def ARM_JOINTS_NUMBER(self) -> int:
        return self._JOINTS_NUMBER[ArmLeg.ARM.value]
    
    @property
    def ARM_NUMBER(self) -> int:
        """
            Number of arms of the robot. 
            It can be 0, 1 (default) or 2.
        """
        return self._LIMB_NUMBER[ArmLeg.ARM.value]
    
    @property # Number of joints of the leg of robot
    def LEG_JOINTS_NUMBER(self) -> int:
        return self._JOINTS_NUMBER[ArmLeg.LEG.value]
    
    @property # Number of legs of the robot (-1, 0, 1 or 2)
    def LEG_NUMBER(self) -> int:
        """
            Number of legs of the robot. 
            It can be -1 (wheeled), 0 (static, default), 1 (left or right leg) or 2 (both legs).
        """
        return self._LIMB_NUMBER[ArmLeg.LEG.value]
    
    @property # Communication interface used by the robot controller, defined by launch argument (return the name of the interface)
    def COMMUNICATION_INTERFACE(self) -> str:
        return self._communication_interface.INTERFACE_NAME
    
    # ATTRIBUTES -------------------------------------------------------------    
    #init_pos = ArticulationDesc() # The initial position of the robot. Should be set to the current position in the constructor (by calling set_init_position_to_current).
    #home_pos = ArticulationDesc() # The home position of the robot (as defined by the Robot). Should be set in the YAML configuration file.

    # TODO: find a more elegant way to set the home position (why the dict is not working?)
    _temp_left_home_pos = ArticulationDesc()
    _temp_right_home_pos = ArticulationDesc()

    init_pos = {
        ArmLeg.ARM.value: 
            {
                ArmLegSide.LEFT.value: None,
                ArmLegSide.RIGHT.value: None
            },
        ArmLeg.LEG.value:
            {
                ArmLegSide.LEFT.value: None,
                ArmLegSide.RIGHT.value: None
            }
    }
    home_pos = init_pos.copy() # The home position of the robot (as defined by the Robot). Should be set in the YAML configuration file.

    # METHODS ----------------------------------------------------------------

    def __init__(self, robot_name) -> None:
        """
            Constructor of the AbstractController class. It initializes the common attributes and calls the Node constructor.
            The robot_name is the name of the robot. It is used to create the node name (by adding "_controller" as suffix).
            The attributes ARM_JOINTS_NUMBER and ARM_NUMBER are declared as parameters in the ROS node. Then, they should be 
            defined in the config file of the robot.

            :param robot_name: (str) the name of the robot.
        """
        self._ROBOT_NAME = robot_name      
        super().__init__(node_name=self.ROBOT_NAME+'_controller')
        self.get_logger().info("========= "+self.ROBOT_NAME+" CONTROLLER =========")

        self.declare_parameter('communication_interface', "ros")
        communication_interface = self.get_parameter('communication_interface').get_parameter_value().string_value
        CommunicationInterfaceClass = globals()[communication_interface.capitalize()+'Interface']
        self._communication_interface = CommunicationInterfaceClass(self)

        self.declare_parameter('simulation_only', False)
        self.simulation_only = self.get_parameter('simulation_only').get_parameter_value().bool_value
        self.get_logger().info(f"Simulation only: {self.simulation_only}")

        self._LIMB_NUMBER = {}
        self._JOINTS_NUMBER = {}
        self.home_pos[ArmLeg.ARM.value] = self.setting_home_position_from_parameter(ArmLeg.ARM)
        self.home_pos[ArmLeg.LEG.value] = self.setting_home_position_from_parameter(ArmLeg.LEG)

        self.create_subscribers()
        self.create_publishers()

    def setting_home_position_from_parameter(self, arm_leg: ArmLeg) -> dict:
        """
            This method is used to set the home position of the robot from the parameters.
            It adapts the home position to the number of limbs and joints of the robot.
            It is the same for the arm and the leg, depending on the input parameter.
            The home position is set in the YAML configuration file.
            The default side is the left side, hence if only one arm is used it will be described as
            left (even if it's a solo arm).

            The home position is set as a list of joint values in degree/radian (depending on the robot). 
            If there is only one limb, the array should contain _JOINTS_NUMBER elements. If there are 
            two limbs, it should contain 2*_JOINTS_NUMBER elements. The first _JOINTS_NUMBER elements 
            are for the left limb, the next _JOINTS_NUMBER elements are for the right limb.
            In the case where the home position is the same for both limbs, the array can contain only 
            _JOINTS_NUMBER elements.

            The yaml configuration file should contain the following parameters:
            - robot_{arm_leg}_number: the number of limbs (-1, 0, 1 or 2 -- wheeled, none, solo (called left), both)
            - robot_{arm_leg}_joints_number: the number of joints of the limb
            - {arm_leg}_robot_home_position: the home position of the limb
            (where {arm_leg} is either 'arm' or 'leg')

            :param arm_leg: (ArmLeg) the arm or leg side to set the home position.

            :return: (dict) the home position of the robot for the given arm/leg side.
        """
        home_pos = {
            ArmLegSide.LEFT.value: None,
            ArmLegSide.RIGHT.value: None
        }
        
        self.declare_parameter(f'robot_{arm_leg.name.lower()}_number', (1 if arm_leg == ArmLeg.ARM else 0))
        self._LIMB_NUMBER[arm_leg.value] = self.get_parameter(f'robot_{arm_leg.name.lower()}_number').get_parameter_value().integer_value

        if (self._LIMB_NUMBER[arm_leg.value] <= 0):
            return home_pos
        
        self.declare_parameter(f'robot_{arm_leg.name.lower()}_joints_number', 0)
        self._JOINTS_NUMBER[arm_leg.value] = self.get_parameter(f'robot_{arm_leg.name.lower()}_joints_number').get_parameter_value().integer_value

        self.declare_parameter(f'{arm_leg.name.lower()}_robot_home_position', [0.0])
        temp_home_pos = self.get_parameter(f'{arm_leg.name.lower()}_robot_home_position').get_parameter_value().double_array_value
        
        if temp_home_pos is not None and len(temp_home_pos) == self._JOINTS_NUMBER[arm_leg.value] * self._LIMB_NUMBER[arm_leg.value]:
            # If home position contain _JOINTS_NUMBER * _LIMB_NUMBER elements, the home position is different for each limb
            # The first _JOINTS_NUMBER elements are for the left limb, the next _JOINTS_NUMBER elements are for the right limb
            self._temp_left_home_pos = temp_home_pos[:self._JOINTS_NUMBER[arm_leg.value]]
            home_pos[ArmLegSide.LEFT.value] = self._temp_left_home_pos
            if self._LIMB_NUMBER[arm_leg.value] == 2:
                self.temp_right_home_pos = temp_home_pos[self._JOINTS_NUMBER[arm_leg.value]:]
                home_pos[ArmLegSide.RIGHT.value] = self._temp_right_home_pos
            else:
                # If a home position is given for both side, but only one limb is defined, the home position is set to None for the right limb
                home_pos[ArmLegSide.RIGHT.value] = None
        elif temp_home_pos is not None and len(temp_home_pos) == self._JOINTS_NUMBER[arm_leg.value]:
            # If home position contain only _JOINTS_NUMBER elements (and have more than one limb), the home position is the same for both
            self._temp_left_home_pos = temp_home_pos
            home_pos[ArmLegSide.LEFT.value] = self._temp_left_home_pos
            home_pos[ArmLegSide.RIGHT.value] = self._temp_left_home_pos
        else:
            # If no home position is set, set to None (shouldn't happen if _LIMIT_NUMBER > 0)
            home_pos[ArmLegSide.LEFT.value] = None
            home_pos[ArmLegSide.RIGHT.value] = None
        
        return home_pos

    # ROS Initialisation methods
    @abstractmethod
    def declare_ros_parameters(self) -> None:
        """
            Declare the ROS parameters needed by the robot controller, and that are specific.
            It should be overriden by the child classes.
        """
        pass

    def create_subscribers(self) -> None:
        """
            Create the subscribers needed by the robot controller. It creates the minimal subscriptions, including:
             * the joint values, 
             * the emergency stop 
             * and the joint values as string.

            It uses the interface selected in the communication_interface parameter. See the package communication_interface for more details.

            It can be overriden by the child classes to add more subscriptions.            
        """
        self._communication_interface.define_subscribers({
            'joint_value_str': (self.joint_print_callback, self._communication_interface.TypeStrAlias),
            'emergency_stop': (self.emergency_stop_callback, self._communication_interface.TypeBoolAlias),
            'joint_value': (self.joints_callback, self._communication_interface.TypeFloatArrayAlias)
        })


    def create_publishers(self) -> None:
        """
            Create the publishers needed by the robot controller. It creates the minimal publishers, including:
             * the joint values to send to the Operator PC.

            It can be overriden by the child classes to add more publishers.
           
            It uses the interface selected in the communication_interface parameter. See the package communication_interface for more details.
        """
        self._communication_interface.define_publishers({
            'robot_joint_values': self._communication_interface.TypeFloatArrayAlias,
            'emergency_stop_status': self._communication_interface.TypeBoolAlias
        })

    # ROS Callback methods
    @abstractmethod
    def joints_callback(self, cmd) -> None:
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
        self.get_logger().info('"%s"' % msg)

    # Robot control methods
    @abstractmethod
    def move_robot(self, joint_values, arm_leg=ArmLeg.ARM, limb_side=ArmLegSide.LEFT) -> None:
        """
            Move the robot to the given joint values. It should be overriden by the child classes.

            :param joint_values: (List[float]) the joint values to move the robot to.
            :param arm_leg: (ArmLeg) the arm or leg side to move. Default is ARM.
            :param limb_side: (ArmLegSide) the arm id to move. Default is LEFT.
        """
        pass

    def move_to_home_position(self) -> None:
        """
            Move the robot to the home position. It calls the move_robot method with the home position.
            It loops over all the limbs, and each side, and move them to the home position.
            The order is the following: left arm, right arm, left leg, right leg.
        """
        for home_pos_limb in self.home_pos:
            for home_pos in home_pos_limb:
                if home_pos is not None:
                    self.move_robot(home_pos)
        #self.move_robot(self.home_pos)

    def move_to_init_position(self) -> None:
        """
            Move the robot to the initial position. It calls the move_robot method with the initial position.
        """
        for init_pos_limb in self.init_pos:
            for init_pos in init_pos_limb:
                if init_pos is not None:
                    self.move_robot(init_pos)
        #self.move_robot(self.init_pos)

    @abstractmethod
    def set_init_position_to_current(self) -> None:
        """
            Set the initial position of the robot to the current position. It should be overriden by the child classes.
        """
        pass