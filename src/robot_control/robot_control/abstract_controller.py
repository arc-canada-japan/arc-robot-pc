from abc import ABC, abstractmethod
import array
import copy
import os
import sys
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from typing import List
from std_msgs.msg import String, Bool, Float32MultiArray
from rclpy.parameter import Parameter, parameter_value_to_python
from enum import Enum
from communication_interface import * # Import all classes from the communication_interface package (no need to use the module name)
import robot_control.controller_tools as ct
import time

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
        return self.limbs_architecture.first_value_of_type(ct.ArmLeg.ARM)
    
    @property
    def ARM_NUMBER(self) -> int:
        """
            Number of arms of the robot. 
            It can be 0, 1 (default) or 2.
        """
        return self.limbs_architecture.arm_count()
    
    @property # Number of joints of the leg of robot
    def LEG_JOINTS_NUMBER(self) -> int:
        return self.limbs_architecture.first_value_of_type(ct.ArmLeg.LEG)
    
    @property # Number of legs of the robot (-1, 0, 1 or 2)
    def LEG_NUMBER(self) -> int:
        """
            Number of legs of the robot. 
            It can be -1 (wheeled), 0 (static, default), 1 (left or right leg) or 2 (both legs).
        """
        return self.limbs_architecture.leg_count()

    @property
    def WHEEL_NUMBER(self) -> int:
        """
            Number of wheels of the robot.
        """
        return self.limbs_architecture.wheel_count()
    
    @property # Number of limbs of the robot
    def LIMB_NUMBER(self) -> int:
        """
            Number of limbs (arm+leg) of the robot. 
            In case of wheel, it will be counted as one limb.
        """
        return self.limbs_architecture.limb_count()
    
    @property # Communication interface used by the robot controller, defined by launch argument (return the name of the interface)
    def COMMUNICATION_INTERFACE(self) -> str:
        return self._communication_interface.INTERFACE_NAME    
   
    # ATTRIBUTES -------------------------------------------------------------    
    jav_init_pos = ct.JointAnglesValues()
    jav_home_pos = ct.JointAnglesValues()
    jav_current_pos = ct.JointAnglesValues()

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

        communication_interface = self.declare_and_get_ros_parameter('communication_interface', "ros")
        CommunicationInterfaceClass = globals()[communication_interface.capitalize()+'Interface']
        self._communication_interface = CommunicationInterfaceClass(self)

        self.simulation_only = self.declare_and_get_ros_parameter('simulation_only', False, log=True)

        self.activation_status              = ct.LimbData("empty", default_value=False, force_same_value_type=True)
        self.activation_status_has_changed  = ct.LimbData("empty", default_value=False, force_same_value_type=True)
        self.limbs_architecture             = ct.LimbData("empty", default_value=0, force_same_value_type=True)
        #self.limbs_joint_value             = ct.LimbData("empty")
        
        self.setting_robot_limbs_from_parameters()
        self.setting_home_position_from_parameter()

        self.create_subscribers()
        self.create_publishers()

    def _init_done(self):
        """
            This method should be called at the end of the init method of the child classes.
            It sets the initialised attribute to True, to indicate that the controller is initialised.
            It also logs a message to indicate that the controller is initialised.
        """
        self.get_logger().info("========= " + self.ROBOT_NAME + " CONTROLLER INIT DONE =========")
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
        self.home_joint_angles_values = ct.LimbData(self.limb_config, default_value=[0.0], force_same_value_type=True)

        for arm_leg in {ct.ArmLeg.ARM, ct.ArmLeg.LEG}:
            limb_number = self.declare_and_get_ros_parameter(f'robot_{arm_leg.name.lower()}_number', (1 if arm_leg == ct.ArmLeg.ARM else 0))
            self.jav_home_pos.set_limb_number(arm_leg, limb_number)
            self.jav_init_pos.set_limb_number(arm_leg, limb_number)
            self.jav_current_pos.set_limb_number(arm_leg, limb_number)

            joint_number = self.declare_and_get_ros_parameter(f'robot_{arm_leg.name.lower()}_joints_number', 0)
            self.jav_home_pos.set_joints_number(arm_leg, joint_number)
            self.jav_init_pos.set_joints_number(arm_leg, joint_number)
            self.jav_current_pos.set_joints_number(arm_leg, joint_number)

            temp_home_pos = self.declare_and_get_ros_parameter(f'{arm_leg.name.lower()}_robot_home_position', [0.0])
            #if temp_home_pos != [0.0]:
            #    self.jav_home_pos.set_limb_articulations_value(arm_leg=arm_leg, value=temp_home_pos)

    def setting_robot_limbs_from_parameters(self):
        """
            Reads and configures the robot's limb and joint settings from ROS parameters.

            For each limb type (arm, leg, wheel, etc.), this method:
            - Retrieves the number of limbs and the number of joints per limb from ROS parameters.
            - Builds the limb configuration accordingly, supporting up to 4 limbs per type.
            - Updates the activation status (`self.activation_status`) with the new configuration.
            - Updates the limb architecture (`self.limbs_architecture`) with the number of joints.
            - Stores the overall limb configuration in `self.limb_config`.

            Raises:
                ValueError: If the number of limbs for a limb type is outside the allowed range [0;4].
        """
        for arm_leg in ct.ArmLeg:
            limb_number = self.declare_and_get_ros_parameter(f'robot_{arm_leg.name.lower()}_number', 0)
            joint_number = self.declare_and_get_ros_parameter(f'robot_{arm_leg.name.lower()}_joints_number', 0)

            if limb_number == 0:
                continue

            if limb_number == 1:
                config = [(arm_leg, ct.ArmLegSide.NONE)]
            elif limb_number == 2:
                config = [(arm_leg, ct.ArmLegSide.LEFT), (arm_leg, ct.ArmLegSide.RIGHT)]
            elif limb_number == 3:
                config = [(arm_leg, ct.ArmLegSide.LEFT), (arm_leg, ct.ArmLegSide.RIGHT), (arm_leg, ct.ArmLegSide.MIDDLE)]
            elif limb_number == 4:
                config = [
                    (arm_leg, ct.ArmLegSide.LEFT),
                    (arm_leg, ct.ArmLegSide.RIGHT),
                    (arm_leg, ct.ArmLegSide.LEFT2),
                    (arm_leg, ct.ArmLegSide.RIGHT2)
                ]
            else:
                raise ValueError(f"The number of {arm_leg.name.lower()} should be in [0;4]")

            self.activation_status.add_limbs(config, False)
            self.limbs_architecture.add_limbs(config,joint_number)
        self.limb_config = self.activation_status.config
        self.activation_status_has_changed = self.activation_status

    # ROS Initialisation methods
    @abstractmethod
    def set_ros_parameters(self) -> None:
        """
            Set the ROS parameters needed by the robot controller, and that are specific.
            It should be overriden by the child classes.
        """
        pass

    def declare_and_get_ros_parameter(self, parameter_name: str, default_value=None, parameter_descriptor=None, log=False):
        """
            Declares a ROS parameter and retrieves its value in Python-native format.

            This function declares a parameter with the given name and default value,
            retrieves its value from the ROS parameter server, converts it to a native
            Python type, and optionally logs the parameter's name and value.

            Args:
                parameter_name (str): The name of the parameter to declare and retrieve.
                default_value (Any, optional): The default value to assign if the parameter is not set. Defaults to None.
                parameter_descriptor (rcl_interfaces.msg.ParameterDescriptor, optional): Descriptor providing metadata about the parameter. Defaults to None.
                log (bool, optional): If True, logs the parameter name and value. Defaults to False.

            Returns:
                Any: The value of the parameter converted to its corresponding Python type.
        """
        try:
            self.declare_parameter(parameter_name, default_value, descriptor=parameter_descriptor)
        except ParameterAlreadyDeclaredException:
            pass
        except Exception as e:
            self.get_logger().error(f"Error while getting parameter {parameter_name} with default value {default_value}: {e}")
        val = self.get_parameter(parameter_name).get_parameter_value()
        val = parameter_value_to_python(val)
        if log: self.get_logger().info(f"{parameter_name}: {val}")

        return val

    def create_subscribers(self) -> None:
        """
            Create the subscribers needed by the robot controller. It creates the minimal subscriptions, including:
             * the joint values, 
             * the emergency stop 
             * and the joint values as string.

            It uses the interface selected in the communication_interface parameter. See the package communication_interface for more details.

            It can be overriden by the child classes to add more subscriptions (but the super class should be called, i.e. this one).            
        """
        self._communication_interface.define_subscribers({
            'joint_value_str': (self.joint_print_callback, self._communication_interface.TypeStrAlias),
            'emergency_stop': (self.emergency_stop_callback, self._communication_interface.TypeBoolAlias),
            'joint_value': (self.joints_callback, self._communication_interface.TypeFloatArrayAlias),
            'end_effector_position': (self.end_effector_position_callback, self._communication_interface.TypeFloatArrayAlias),
            'hand_open_close': (self.hand_open_close_callback, self._communication_interface.TypeFloatArrayAlias),
            'controller_activated': (self.controller_activated_callback, self._communication_interface.TypeFloatArrayAlias),
            'current_timestamp_operator_ms': (self.current_time_operator_callback, self._communication_interface.TypeIntAlias)
        })


    def create_publishers(self) -> None:
        """
            Create the publishers needed by the robot controller. It creates the minimal publishers, including:
             * the joint values to send to the Operator PC.

            It can be overriden by the child classes to add more publishers (but the super class should be called, i.e. this one).
           
            It uses the interface selected in the communication_interface parameter. See the package communication_interface for more details.
        """
        self._communication_interface.define_publishers({
            'robot_joint_values': self._communication_interface.TypeFloatArrayAlias,
            #'robot_end_effector_position': self._communication_interface.TypeFloatArrayAlias,
            'emergency_stop_status': self._communication_interface.TypeBoolAlias,
            'current_timestamp_robot_ms': self._communication_interface.TypeIntAlias
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
    def hand_open_close_callback(self, cmd) -> None:
        """
            Callback method for the gripper/hand state (open or close). It should be overriden by the child classes.
            This method is called when the robot receives new gripper state. It should open or close the hand gripper accordingly.

            :param cmd: (Float32MultiArray) the hand gripper state (from 0.0 to 1.0). 0.0 is close, 1.0 is open. The table contain two values: the arm_id and the hand state.
        """
        pass

    #@abstractmethod
    def controller_activated_callback(self, cmd) -> None:
        """
            Callback method for the activation of the controller (the movement shouldn't be taken into account if false).
            This method is called when the robot receives new controller activation state. It should move or not the robot accordingly.

            The input value is eithr an array of two element ([side ; state]), assuming the limb if the robot has only one type, or an array of three elements ([limb ; side ; state])

            :param cmd: (Float32MultiArray) the controller activation state (0.0 or 1.0). 0.0 is not disabled, 1.0 is enabled.
        """
        activation = cmd
        if len(activation)==2: # only one kind of limb
            side = ct.ArmLegSide(int(activation[0]))
            index = side
        elif len(activation==3):
            limb = ct.ArmLeg(int(activation[0]))
            side = ct.ArmLegSide(int(activation[1]))
            index = (limb, side)
        else:
            self.get_logger().error(f"Activation data in wrong format, it should be an array of two or three values. Received: {new_status}. Operation aborted")
            return
        new_status = (activation[-1] == 1.0) # the value is the last

        prev_status = self.activation_status[index]
        self.activation_status[index] = new_status

        if prev_status != new_status:
            self.get_logger().info(f"Activation button pressed: {new_status}")
            self.activation_status_has_changed[index] = True
        else:
            self.activation_status_has_changed[index] = False

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

    def current_time_operator_callback(self, msg) -> None:
        """
            Callback method for the current time of the operator. It computes the delay between the robot and the operator.
        
            :param msg: (Int) the current time of the operator as a POSIX timestamp (in millisecond).
        """
        operator_time = msg
        robot_time = round(time.time_ns() / 10**6)
        time_diff = robot_time - operator_time
        self.get_logger().info(f"Delay: {time_diff} ms")

        # Publish robot time
        self._communication_interface.publish('current_timestamp_robot_ms', robot_time)

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
                    self.move_robot(limb_side_val, False, limb, limb_side)

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

    def extract_joint_angle_command(self, cmd):
        """
            Extracts and organizes joint angle commands for each limb from the received data (array of float in cmd.data).

            The method supports two scenarios:
            1) If there is only one limb in the architecture, the data is assumed to contain only joint values (no limb code).
            2) Otherwise, the data format is [LIMB_CODE, DATA_1, DATA_2, ..., LIMB_CODE, DATA_1, ...] for each limb.
            Each limb's data is identified by a 2-digit limb code (ArmLeg*10 + ArmLegSide).

            Args:
                cmd (any): An object containing a 'data' attribute with a list of float values.
                    This list encodes the joint commands for all limbs.

            Returns:
                LimbData: A LimbData instance containing the joint angle commands for each limb.

            Raises:
                ValueError: If the data format is inconsistent with the robot's current architecture
                            or if a limb code cannot be parsed.
        """
        if len(self.limbs_architecture) ==1 and len(cmd.data) == next(iter(self.limbs_architecture.values())): # in case of only one limb, no need for code
            return ct.LimbData(self.limb_config, cmd.data, True)
        
        data_len = sum((v+1) for (_, _), v in self.limbs_architecture.items()) # +1 because of the limb code
        cmd = cmd.data

        if len(cmd) != data_len:
            raise ValueError("The received command is not consistant with the current robot architecture.")
        
        data = ct.LimbData(self.limb_config, [0.0], True) # Create the empty data structure
        i = 0
        while i < len(cmd):
            code = f"{int(cmd[i]):02d}"
            try:
                limb = ct.ArmLeg(int(code[:1]))
                side = ct.ArmLegSide(int(code[1:]))
            except ValueError:
                raise ValueError(f"Joint angle code format wrong ({code})")

            length = self.limbs_architecture[(limb, side)]
            data[(limb, side)] = cmd[i+1:i+1+length]
            i += 1 + length  # move to the next limb code

        return data
    
    def append_sys_path_from_param(self, param_name):
        self.declare_parameter("paths." + param_name, "")
        path = self.get_parameter("paths." + param_name).get_parameter_value().string_value
        if path and os.path.exists(path):
            sys.path.append(path)
        else:
            self.get_logger().warn(f"Path from {param_name} not found or doesn't exist: {path}")


    # @abstractmethod
    # def set_home_position_from_end_effector_position(self, ee_position) -> None:
    #     """
    #         Set the home position of the robot to the current end effector position. It should be overriden by the child classes.
    #     """
    #     pass
