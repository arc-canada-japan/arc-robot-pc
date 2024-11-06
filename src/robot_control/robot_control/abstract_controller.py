from abc import ABC, abstractmethod
import array
import copy
from rclpy.node import Node
from typing import List
from std_msgs.msg import String, Bool, Float32MultiArray
from enum import Enum
from communication_interface import * # Import all classes from the communication_interface package (no need to use the module name)
import robot_control.controller_tools as ct

class AbstractController(ABC, Node):
    """
        Abstract class for the robot controller. It defines the common methods and attributes.
        All the controllers must inherit from this class. For basic functionnalities, the definition
        from this abstract class can be used. For specific functionnalities, the methods can be overriden.

        The communication is done via the communication_interface ROS package. The interface used is defined
        by the launch argument 'communication_interface'. It allows the package to be versatile, and use different
        communication interfaces (ROS, ZMQ, etc.) without changing the code.

        The variables are prefixed with:
            * 'jav_' are Joint Angles Values (as a ct.JointAnglesValuesn ~ a list[float] for each limb).
            * 'eec_' are End Effector Coordinates (as a list[float] of 6 components: x,y,z,roll,pitch,yaw).
        The controller can use both depending on the robot API and purpose. The values are for the robot,
        except if specified otherwise (e.g. for the controller).
    """
    # READ-ONLY ATTRIBUTES ---------------------------------------------------
    @property # Name of the robot
    def ROBOT_NAME(self) -> str:
        return self._ROBOT_NAME
    
    @property # Number of joints of the arm of robot
    def ARM_JOINTS_NUMBER(self) -> int:
        return self.jav_home_pos.arm_joints_number
    
    @property
    def ARM_NUMBER(self) -> int:
        """
            Number of arms of the robot. 
            It can be 0, 1 (default) or 2.
        """
        return self.jav_home_pos.arm_limb_number
    
    @property # Number of joints of the leg of robot
    def LEG_JOINTS_NUMBER(self) -> int:
        return self.jav_home_pos.leg_joints_number
    
    @property # Number of legs of the robot (-1, 0, 1 or 2)
    def LEG_NUMBER(self) -> int:
        """
            Number of legs of the robot. 
            It can be -1 (wheeled), 0 (static, default), 1 (left or right leg) or 2 (both legs).
        """
        return self.jav_home_pos.leg_limb_number
    
    @property # Communication interface used by the robot controller, defined by launch argument (return the name of the interface)
    def COMMUNICATION_INTERFACE(self) -> str:
        return self._communication_interface.INTERFACE_NAME
    
    # ATTRIBUTES -------------------------------------------------------------    
    jav_init_pos = ct.JointAnglesValues()
    jav_home_pos = ct.JointAnglesValues()
    jav_current_pos = ct.JointAnglesValues()

    eec_controller_pos = None
    eec_current_pos = None

    initialised = False

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
        
        self.setting_home_position_from_parameter()

        self.create_subscribers()
        self.create_publishers()

    def _init_done(self):
        """
            This method should be called at the end of the init method of the child classes.
            It sets the initialised attribute to True, to indicate that the controller is initialised.
            It also logs a message to indicate that the controller is initialised.
        """
        self.get_logger().info("========= "+self.ROBOT_NAME+" CONTROLLER INIT DONE =========")
        self.initialised = True

    def setting_home_position_from_parameter(self):
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

            It also sets the number of limbs and joints of the robot for the init_pos and current_joint_pos.

            The yaml configuration file should contain the following parameters:
            - robot_{arm_leg}_number: the number of limbs (-1, 0, 1 or 2 -- wheeled, none, solo (called left), both)
            - robot_{arm_leg}_joints_number: the number of joints of the limb
            - {arm_leg}_robot_home_position: the home position of the limb
            (where {arm_leg} is either 'arm' or 'leg')

        """
        for arm_leg in {ct.ArmLeg.ARM, ct.ArmLeg.LEG}:
            self.declare_parameter(f'robot_{arm_leg.name.lower()}_number', (1 if arm_leg == ct.ArmLeg.ARM else 0))
            limb_number = self.get_parameter(f'robot_{arm_leg.name.lower()}_number').get_parameter_value().integer_value
            self.jav_home_pos.set_limb_number(arm_leg, limb_number)
            self.jav_init_pos.set_limb_number(arm_leg, limb_number)
            self.jav_current_pos.set_limb_number(arm_leg, limb_number)

            self.declare_parameter(f'robot_{arm_leg.name.lower()}_joints_number', 0)
            joint_number = self.get_parameter(f'robot_{arm_leg.name.lower()}_joints_number').get_parameter_value().integer_value
            self.jav_home_pos.set_joints_number(arm_leg, joint_number)
            self.jav_init_pos.set_joints_number(arm_leg, joint_number)
            self.jav_current_pos.set_joints_number(arm_leg, joint_number)

            self.declare_parameter(f'{arm_leg.name.lower()}_robot_home_position', [0.0])
            temp_home_pos = self.get_parameter(f'{arm_leg.name.lower()}_robot_home_position').get_parameter_value().double_array_value
            if temp_home_pos != [0.0]:
                self.jav_home_pos.set_limb_articulations_value(arm_leg=arm_leg, value=temp_home_pos)

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
            'joint_value': (self.joints_callback, self._communication_interface.TypeFloatArrayAlias),
            'end_effector_position': (self.end_effector_position_callback, self._communication_interface.TypeFloatArrayAlias)
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
            #'robot_end_effector_position': self._communication_interface.TypeFloatArrayAlias,
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
    def end_effector_position_callback(self, cmd) -> None:
        """
            Callback method for the end effector position. It should be overriden by the child classes.
            This method is called when the robot receives new end effector position. It should move the robot's end effector to this position.
            Using either Inverse Kinematics or robot API built-in function, the robot should move to the new position.

            :param cmd: (Float32MultiArray) the new end effector position received by the robot. The position is received as a list of 3 float values,
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
    def move_robot(self, pos_or_joint_values, position=False, arm_leg=ct.ArmLeg.ARM, limb_side=ct.ArmLegSide.LEFT, wait=False) -> None:
        """
            Move the robot to the given joint values. It should be overriden by the child classes.

            :param pos_or_joint_values: (List[float]) the joint values or end effector position to move the robot to.
            :param position: (Bool) if True, the pos_or_joint_values is an end effector position. If False, it is joint values.
            :param arm_leg: (ct.ArmLeg) the arm or leg side to move. Default is ARM.
            :param limb_side: (ct.ArmLegSide) the arm id to move. Default is LEFT.
        """
        pass

    def move_to_home_position(self) -> None:
        """
            Move the robot to the home position. It calls the move_robot method with the home position.
            It loops over all the limbs, and each side, and move them to the home position.
            The order is the following: left arm, right arm, left leg, right leg.
        """
        self.__move_to(self.jav_home_pos)

    def move_to_init_position(self) -> None:
        """
            Move the robot to the initial position. It calls the move_robot method with the initial position.
        """
        self.__move_to(self.jav_init_pos)

    def __move_to(self, position):
        for limb, limb_val in position:
            for limb_side, limb_side_val in limb_val.items():
                if limb_side_val is not None:
                    self.move_robot(limb_side_val, limb, limb_side)

    @abstractmethod
    def set_init_position_to_current(self) -> None:
        """
            Set the initial position of the robot to the current position. It should be overriden by the child classes.
        """
        pass

    @abstractmethod
    def _transform_unity_to_robot(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the Unity coordinate system to the robot coordinate system.
        """
        pass

    @abstractmethod
    def _transform_robot_to_unity(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the robot coordinate system to the Unity coordinate system.
        """
        pass

    # @abstractmethod
    # def set_home_position_from_end_effector_position(self, ee_position) -> None:
    #     """
    #         Set the home position of the robot to the current end effector position. It should be overriden by the child classes.
    #     """
    #     pass