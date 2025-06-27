from typing import List
import rclpy
import sys
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from robot_control.abstract_controller import AbstractController
import robot_control.controller_tools as ct

from omniORB import CORBA
sys.path.append("/opt")
sys.path.append("/opt/NxApiLib/idl_NxApi")
from NxApiLib import any as ANY
from NxApi import *
import numpy as np

# TODO NOT FINISHED

class FillieController(AbstractController):
    """
        Controller class for the Nextage robot, adapted for the Fillie configuration.

        This controller receives joint commands and end effector positions from an operator PC via ROS,
        and it uses the NxApi (CORBA-based API) to send commands to the robot. It also handles feedback
        (e.g., limits, offsets, gains) and safety mechanisms (e.g., emergency stop).

        Key features:
        - Establishes a CORBA connection to the robot and sets up the API.
        - Reads and configures ROS parameters for robot control (IP, speed, etc.).
        - Supports different control sources (Unity, keyboard), with appropriate coordinate transformations.
        - Provides methods to process joint angle commands, end effector commands, and hand gripper commands.
        - Tracks the active limbs, their states, and uses a `LimbData` structure for flexibility.
        - Handles feedback limits, offset corrections, and gain adjustments to ensure smooth robot operation.

        Attributes:
            controller: The CORBA API controller object for sending commands to the robot.
            robot: The main robot API object.
            com_angle_var: API variable for commanded joint angles.
            act_angle_var: API variable for actual joint angles.
            arms: A LimbData structure holding references to the arm API objects.
            arm_pos_command_var: A LimbData structure holding the position command variables for each arm.
            arm_pos_actual_var: A LimbData structure holding the actual position variables for each arm.
            speed_var: API variable for the robot's movement speed.
            param_wait_motion: Pre-built CORBA parameter for motion wait time.
            activation_status: LimbData for tracking which limbs are active.
            activation_status_has_changed: LimbData for tracking changes in activation status.
            hand_current_status: LimbData for tracking the current status of each hand gripper.
            LIMITS: LimbData for position limits of each limb.
            offset: LimbData for offset corrections for each limb.
            GAINS: LimbData for gain settings of each limb.
            mirrored: Whether to mirror the control (used in Unity transformation).
            control_source: The source of control commands ("unity" or "keyboard").
            omit_orientation: Whether to ignore orientation data during movement.

        Notes:
            - The controller automatically acquires API authority on initialization.
            - It switches the robot servos on during initialization.
            - This implementation assumes the robot has either one or two arms and does not handle legs or wheels (but can be extended).
    """

    def __init__(self):
        """
            Initializes the FillieController.

            - Sets up ROS parameters and default configurations.
            - Connects to the NxApi CORBA server for direct robot control.
            - Initializes data structures for controlling arms, limits, offsets, and gains.
            - Defines the transformation matrices based on the chosen control source.
            - Optionally initializes the hand grippers if enabled.

            Raises:
                SystemExit: If connection to the robot API server fails.
        """
        super().__init__(robot_name="fillie")

        self.set_ros_parameters()

        # if not self.simulation_only:
        #     self.use_hand = True            
        # else:
        #     self.init_pos = self.home_pos
        #     self.ip = "127.0.0.1"
        #     self.use_hand = False

        argv = [ '-ORBdefaultWCharCodeSet',
                    'UTF-16',
                    '-ORBgiopMaxMsgSize',
                    '104857600']
            
        try:
            # Connect to the API server
            orb = CORBA.ORB_init(argv, CORBA.ORB_ID)
            api = orb.string_to_object(f"corbaloc:iiop:1.2@{self.ip_robot}:2809/RootControllerApi")
            root_controller = api._narrow(RootNxController)
            # Raising an exception if the maximum number of clients is reached
            self.controller = root_controller.GetNxController("", "")
            # Acquire API Authority
            self.controller.Execute("GetAuthority", ANY.to_any(None))
        except Exception as e:
            self.get_logger().error("Impossible to connect to the robot: " + str(e))
            exit(10)

        # Robot objects
        self.robot = self.controller.GetNxObject("Robot", "All")
        self.robot.Execute("ServoOn", ANY.to_any(None))
        self.com_angle_var = self.robot.GetVariable("@JOINT_ANGLE", "")
        self.act_angle_var = self.robot.GetVariable("@ACTUAL_JOINT_ANGLE", "")

        # Arm-specific objects and variables
        self.arms = ct.LimbData(self.limb_config)
        self.arms[ct.ArmLegSide.LEFT] = self.controller.GetNxObject("Robot", "LArm")
        self.arms[ct.ArmLegSide.RIGHT] = self.controller.GetNxObject("Robot", "RArm")

        self.arm_pos_command_var = ct.LimbData(self.limb_config)
        self.arm_pos_actual_var = ct.LimbData(self.limb_config)

        for side, arm in self.arms.items():
            self.arm_pos_command_var[side] = arm.GetVariable("@BASE_POSITION", "")
            self.arm_pos_actual_var[side] = arm.GetVariable("@ACTUAL_BASE_POSITION", "")

        # Set production speed
        self.speed_var = self.controller.GetVariable("@SPEED", "")
        self.speed_var.Execute("SetValues", ANY.to_any([[["value", self.robot_speed]]]))

        self.param_wait_motion = ANY.to_any([["queueSize", self.robot_wait_queue_size]])

        self._define_transformation_matrix()
        
        self.set_init_position_to_current() # Check if useful
        self.send_current_joint_value()

        #init hand
        if self.use_hand:
            self.hand_current_status = ct.LimbData(self.limb_config)
            param_hand = ANY.to_any([["command", "ARH_Initialize"], ["arg", "1,COM29"]])
            self.controller.Execute("RunCommand", param_hand)
            param_hand = ANY.to_any([["command", "ARH_Initialize"], ["arg", "2,COM30"]])
            self.controller.Execute("RunCommand", param_hand)

        self.activation_initial_pos = ct.LimbData(self.limb_config)

        self._init_done()  

    def __del__(self):
        """
            Called when the controller is destroyed.

            - Turns off the robot's servo.
            - Releases API authority from the robot controller.
        """
        if not self.simulation_only and hasattr(self, 'robot') and self.robot is not None:
            self.robot.Execute("ServoOff", ANY.to_any(None))
            # Release API authority
            self.controller.Execute("ReleaseAuthority", ANY.to_any(None))

    def set_ros_parameters(self):
        """
            Declares and retrieves ROS parameters needed for the controller.

            Parameters include:
            - Robot IP address.
            - Speed and wait queue size.
            - Control source ("unity" or "keyboard").
            - Hand gripper usage and limits.
            - Position offsets and gains for each arm.
            - Home position of the robot.
        """
        # Get the robot IP address from the parameter send by the launch file
        self.ip_robot = self.declare_and_get_ros_parameter('robot_ip', '192.168.128.55')
        #self.ip_rmt = self.declare_and_get_ros_parameter('robot_ip', '192.168.128.55')
        self.robot_speed = self.declare_and_get_ros_parameter('robot_speed', 100)
        self.robot_wait_queue_size = self.declare_and_get_ros_parameter('robot_wait_queue_size', 2)
        self.mirrored = self.declare_and_get_ros_parameter('mirrored', False)
        self.control_source = self.declare_and_get_ros_parameter('control_source', "unity").lower()
        self.use_hand = self.declare_and_get_ros_parameter('use_hand', True)

        self.LIMITS = ct.LimbData(self.limb_config)
        self.LIMITS[ct.ArmLegSide.LEFT] = np.array(self.declare_and_get_ros_parameter('limit_left', [0.0, 0.0] * 6)).reshape((6, 2))
        self.LIMITS[ct.ArmLegSide.RIGHT] = np.array(self.declare_and_get_ros_parameter('limit_right', [0.0, 0.0] * 6)).reshape((6, 2))

        self.offset = ct.LimbData(self.limb_config)
        self.offset[ct.ArmLegSide.LEFT] = ct.EndEffectorCoordinates(self.declare_and_get_ros_parameter('offset_left', [0.0] * 6))
        self.offset[ct.ArmLegSide.RIGHT] = ct.EndEffectorCoordinates(self.declare_and_get_ros_parameter('offset_right', [0.0] * 6))
        
        self.GAINS = ct.LimbData(self.limb_config)
        self.GAINS[ct.ArmLegSide.LEFT] = ct.EndEffectorCoordinates(self._expand_gain(self.declare_and_get_ros_parameter('gain_left', 0.5)))
        self.GAINS[ct.ArmLegSide.RIGHT] = ct.EndEffectorCoordinates(self._expand_gain(self.declare_and_get_ros_parameter('gain_right', 0.5)))

        self.omit_orientation = self.declare_and_get_ros_parameter('omit_orientation', False)
        # Check if useful and/or adapt
        #self.declare_parameter('robot_home_position', self.home_pos)
        #self.home_pos = self.get_parameter('robot_home_position').get_parameter_value()._double_array_value

    def _expand_gain(self, value):
        """
            Expands a single gain value or a list of 6 values to ensure a valid gain vector for each joint.

            Args:
                value (float or list/tuple of 6 floats): The gain value(s) to use.

            Returns:
                list[float]: A list of 6 float gain values.

            Raises:
                ValueError: If the input is not a number or a list/tuple of 6 numbers.
        """
        if isinstance(value, (int, float)):
            return [float(value)] * 6
        elif isinstance(value, (list, tuple)) and len(value) == 6 and all(isinstance(v, (int, float)) for v in value):
            return list(map(float, value))
        else:
            raise ValueError(f"Gain must be a number or a list/tuple of 6 numbers, got: {value}")

    def emergency_stop_callback(self, msg):
        """
            This function is called when an emergency stop is requested.
            It stops the robot if the message is True.
        """
        self.get_logger().info(f"Emergency stop: {msg}")
        if self.simulation_only: # No need to stop the simulation
            return
        if msg:
            #TODO stop
            pass
        else:
            #TODO reset
            pass

    def _define_transformation_matrix(self):
        """
            Defines the transformation matrix (and its inverse) based on the control source.

            - For "unity", sets a transformation that accounts for mirroring if enabled.
            - For "keyboard", sets a fixed transformation matrix.
            - Raises an error if an unsupported control source is provided.
        """
        transformation_map = {
            "unity": self._unity_transformation,
            "keyboard": self._keyboard_transformation
        }

        if self.control_source not in transformation_map:
            valid_sources = list(transformation_map.keys())
            raise ValueError(
                f"Control source parameter ({self.control_source}) is invalid. "
                f"Valid options are: {valid_sources}"
            )

        # Call the appropriate method to set the transformation matrix
        transformation_map[self.control_source]()

        # Compute inverse
        self.inv_transformation_matrix = np.linalg.inv(self.transformation_matrix)

    def _unity_transformation(self):
        """
            Defines the transformation matrix for the Unity control source.

            Uses different matrices based on whether mirroring is enabled.
        """
        if self.mirrored:
            self.transformation_matrix = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])  # mirrored
        else:
            self.transformation_matrix = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]])  # normal

    def _keyboard_transformation(self):
        """
            Defines the transformation matrix for the keyboard control source.
        """
        self.transformation_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])

    def _transform_unity_to_robot(self, cmd):
        """
            Transforms an end effector position and orientation command from Unity coordinates
            to the robot's coordinate system.

            Args:
                cmd (list[float]): The input command with 6 values (position + rotation).

            Returns:
                np.ndarray: The transformed command, with position converted to millimeters.
        """
        cmd = np.array(cmd)
        trans = np.dot(self.transformation_matrix, cmd[:3]) * 1000 # Apply the transformation matrix to have the good coordinate for the end effector position, plus convert metre to millimetre
        rot = np.dot(self.transformation_matrix, cmd[3:]) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, rot])
        return cmd
    
    def _transform_robot_to_unity(self, cmd):
        """
            Transforms an end effector command from the robot's coordinates to Unity coordinates.

            Args:
                cmd (list[float]): The input command with 6 values (position + rotation).

            Returns:
                np.ndarray: The transformed command, with position converted to meters.
        """
        cmd = np.array(cmd)
        trans = np.dot(self.inv_transformation_matrix, cmd[:3]) / 1000 # Apply the transformation matrix to have the good coordinate for the end effector position, plus convert milliter to meter
        rot = np.dot(self.inv_transformation_matrix, cmd[3:]) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, rot])
        return cmd

    def joints_callback(self, cmd):
        """
            This function is called when a new joint value is received from the Operator PC.
            It computes the robot joint values regarding the initial position of the robot,
            and it moves the robot to the new joint values.

            This function is currently a placeholder.
        """
        pass

    def end_effector_position_callback(self, cmd):
        """
            Callback method for receiving end effector positions from the operator PC.

            - Validates the message length.
            - Transforms the command to robot coordinates.
            - Applies offset corrections and gain scaling.
            - Applies limits to ensure safe motion.
            - Moves the robot to the new position if valid.

            Args:
                cmd: The ROS message containing the 12-value end effector positions.
        """
        if not self.initialised:
            return
        if len(cmd.data) != 12:
            self.get_logger().error(f"End effector position message is expecting 12 values, {len(cmd.data)} have been received. Operation aborted")
            return
        
        inputs = {
            ct.ArmLegSide.LEFT: cmd.data[0:6],
            ct.ArmLegSide.RIGHT: cmd.data[6:12]
        }
        enabled_sides = []
        for side in (ct.ArmLegSide.LEFT, ct.ArmLegSide.RIGHT):
            if inputs[side] != [-1.0] * self.jav_home_pos.arm_joints_number:
                enabled_sides.append(side)
            inputs[side] = ct.EndEffectorCoordinates(self._transform_unity_to_robot(inputs[side]))
        if len(enabled_sides) == 0:
            self.get_logger().warn("Received empty data for both arms. Operation aborted")
            return
        
        for side in enabled_sides:
            if not self.activation_status[side]:
                break
            position = ct.EndEffectorCoordinates(ANY.from_any(self.arm_pos_command_var[side].Execute("GetValues", ANY.to_any(None)))[0])
            position_opposite = ct.EndEffectorCoordinates(ANY.from_any(self.arm_pos_command_var[self._opposite(side)].Execute("GetValues", ANY.to_any(None)))[0])
            output_command = ct.EndEffectorCoordinates()
            a_list, b_list = zip(*self.LIMITS[side])
            limits_a = ct.EndEffectorCoordinates(list(a_list))
            limits_b = ct.EndEffectorCoordinates(list(b_list))

            if self.activation_initial_pos[side] is None or self.activation_status_has_changed[side]:
                self.activation_initial_pos[side] = inputs[side]
                self.offset[side] = position - inputs[side]
                self.get_logger().info(f"{side} arm acivated and reset position")
            output_command = self.activation_initial_pos[side] + (inputs[side] - self.activation_initial_pos[side]) * self.GAINS[side] + self.offset[side]
            temp_copy = output_command
            output_command = output_command.max_elements(limits_a).min_elements(limits_b)

            offset = 200 if side == ct.ArmLegSide.LEFT else -200
            output_command.y = min(output_command.y, position_opposite.y + offset) # avoid crossing 2 arms

            for i in range(len(output_command.coordinates)):                
                if temp_copy[i] != output_command[i]:
                    self.activation_initial_pos[side][i] = inputs[side][i]
                    self.offset[side][i] = position[i] - inputs[side][i]
                    self.get_logger().info(f"Axis {output_command.el_name(i)} exceed the limit, reset to {output_command[i]}")

            if self.omit_orientation:
                output_command.orientation = [0.0]*3

            self.move_robot(output_command.to_list(), True, limb_side=side)
    
    def _opposite(side: ct.ArmLegSide):
        """
            Returns the opposite side for a given ArmLegSide (LEFT <-> RIGHT).

            Args:
                side (ArmLegSide): The current side.

            Returns:
                ArmLegSide: The opposite side.

            Raises:
                ValueError: If the side is NONE.
        """
        if side == ct.ArmLegSide.NONE:
            raise ValueError("Side Undefined")
        return ct.ArmLegSide.RIGHT if side == ct.ArmLegSide.LEFT else ct.ArmLegSide.LEFT

        
    def hand_open_close_callback(self, cmd) -> None:
        """
            Callback for updating the hand gripper's state (open/close).

            Args:
                cmd: A Float32MultiArray with [arm_id, hand state (0.0=closed, 1.0=open)].
            
            - Updates internal status and sends commands to the robot's API.
        """
        if not self.initialised or not self.use_hand:
            return

        hand = cmd.data
        side = int(hand[ct.ArmLegData.ARM_LEG_ID])
        new_status = hand[ct.ArmLegData.DATA]
        current_status = self.hand_current_status[side]

        # First-time setup
        if current_status is None:
            self.hand_current_status[side] = new_status
        # Skip if status hasn't changed
        elif current_status == new_status:
            return

        # Update status and execute grip command
        self.hand_current_status[side] = new_status

        param_hand = ANY.to_any([["command", "ARH_AutoGrip"], ["arg", f"{side},pos={int(new_status * 1000)}"]])
        self.controller.Execute("RunCommand", param_hand)

        param_hand = ANY.to_any([["command", "ARH_WaitBusy"], ["arg", f"{side}"]])
        self.controller.Execute("RunCommand", param_hand)
        
    # def controller_activated_callback(self, cmd) -> None:
    #     """
    #         Callback method for the activation of the controller (the movement shouldn't be taken into account if false).
    #         This method is called when the robot receives new controller activation state. It should move or not the robot accordingly.

    #         :param cmd: (Float32MultiArray) the controller activation state (0.0 or 1.0). 0.0 is not disabled, 1.0 is enabled. The table contain two values: the arm_id and the controller activation state.
    #     """
    #     activation = cmd.data
    #     side = activation[ct.ArmLegData.ARM_LEG_ID]

    #     if side in (ct.ArmLegSide.LEFT, ct.ArmLegSide.RIGHT):
    #         index = side - 1
    #         prev_status = self.activation_status[index]
    #         new_status = (activation[ct.ArmLegData.DATA] == 1.0)
    #         self.activation_status[index] = new_status

    #         if prev_status != new_status:
    #             self.get_logger().info(f"Activation button pressed: {new_status}")
    #     else:
    #         self.get_logger().warn(f"Activation status undefined ({activation})")

    def move_robot(self, pos_or_joint_values, position=False, arm_leg=ct.ArmLeg.ARM, limb_side=ct.ArmLegSide.LEFT, wait=False):
        """
            Sends a movement command to the robot.

            Args:
                pos_or_joint_values (list[float]): Target position or joint angles.
                position (bool): If True, treat the values as Cartesian positions.
                arm_leg (ArmLeg): The limb type to move (default: ARM).
                limb_side (ArmLegSide): The side of the limb (default: LEFT).
                wait (bool): If True, wait for the motion to complete.

            Notes:
                Currently only supports End Effector position movements.
        """
        if position:
            param = ANY.to_any([
                ["comp", 2],
                ["csNo", 0],
                ["csOffset", pos_or_joint_values],
                ["next",True],
                ["speed", self.robot_speed]])
            ret_any = self.arms[limb_side].Execute("Move", param)
            self.arms[limb_side].Execute("WaitMotion", self.param_wait_motion)
            ret = ANY.from_any(ret_any)
            self.get_logger().info(f"Command executed: {pos_or_joint_values}")
            self.get_logger().info(f"Ret: {ret[0]} ; drive time: {ret[1]} ; status: {ret[2]}")
        else:
            pass

        self.send_current_joint_value()

    def set_init_position_to_current(self):
        """
            Placeholder to set the initial robot position to the current state.
        """
        pass

    def send_current_joint_value(self):
        """
            Publishes the current joint values to the communication interface.
        """
        angles = ANY.from_any(self.com_angle_var.Execute("GetValues", ANY.to_any(None)))[0]
        self._communication_interface.publish("robot_joint_values", list(angles))

def main(args=None):
    rclpy.init(args=args)

    controller = FillieController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()