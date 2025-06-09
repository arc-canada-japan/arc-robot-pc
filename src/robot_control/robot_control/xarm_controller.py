from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from robot_control.abstract_controller import AbstractController
from control_msgs.msg import GripperCommand
import robot_control.controller_tools as ct
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from xarm.wrapper import XArmAPI

import time

class XarmController(AbstractController):
    """
        This class is used to controller the xArm robot.
        It receives the joint values from the Operator PC and moves the robot to the new joint values.
        The values are received as a list of 6 float values, representing the joint angles in degree.
    """

    def __init__(self):
        super().__init__(robot_name="xarm")

        self.declare_ros_parameters()

        self.TRANS_SPEED = 1 # mm/s
        self.ROT_SPEED = 10 # r/min
        self.ROT_SPEED = self.ROT_SPEED * ct.RPM_TO_DEG_S # conversion to °/s 
        self.END_EFFECTOR_EPSILON = 3 # cm
        self.MOVE_FACTOR = 1 # multiply the user's controller position by this factor to get the robot position

        # Setting the defaut values for the robot (since xArm is only one arm, the default values are for the left arm) to access the value simply with ".value"
        self.jav_home_pos.default_limb      = ct.ArmLeg.ARM
        self.jav_home_pos.default_side      = ct.ArmLegSide.LEFT
        self.jav_init_pos.default_limb      = ct.ArmLeg.ARM
        self.jav_init_pos.default_side      = ct.ArmLegSide.LEFT
        self.jav_current_pos.default_limb   = ct.ArmLeg.ARM
        self.jav_current_pos.default_side   = ct.ArmLegSide.LEFT

        self.transformation_matrix = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]]) # Transformation matrix from Unity to robot coordinate system
        self.inv_transformation_matrix = np.linalg.inv(self.transformation_matrix) # Inverse transformation matrix from Unity to robot coordinate system

        if not self.simulation_only:
            try:
                self.arm = XArmAPI(self.ip, is_radian=False)
            except Exception as e:
                self.get_logger().error("Impossible to connect to the robot: " + str(e))
                self.arm = None
                exit(10)

            if (ret := self.arm.motion_enable(enable=True)) != 0:
                self.get_logger().error(f"Impossible to enable the robot: {ret}")
            self.arm.clean_error()
            self.arm.clean_warn()

            self.ee_cmd = None
            self.initialised = True

            self.set_init_position_to_current()
            self.move_robot(self.jav_home_pos.value, wait=True) 

            self.initialised = False

            self.arm.set_mode(1)
            self.arm.set_state(state=0)
            time.sleep(1)            

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            # self._communication_interface.modify_qos_for_topic('end_effector_position', qos_profile)

            # Timers for streaming commands and preventing lock
            self.create_timer(0.005, lambda: self.move_robot(None, True)) # TODO cst in config?
            self.create_timer(1, self._anti_lock_timer_callback)

        else:
            self.jav_init_pos.value = self.jav_home_pos.value
            self.jav_current_pos.value = self.jav_home_pos.value
            self.eec_current_pos = ct.EndEffectorCoordinates()

        self.get_logger().info(f"init_pos: {self.jav_init_pos}")
        self.get_logger().info(f"home_pos: {self.jav_home_pos}")

        self._init_done()

    def __del__(self):
        if not self.simulation_only and self.arm is not None:
            self.arm.set_state(4)
            self.arm.disconnect()

    def declare_ros_parameters(self):
        """
            Declare the ROS parameters used by the controller.
        """
        # Get the robot IP address from the parameter send by the launch file
        self.declare_parameter('robot_ip', '192.168.1.217')
        self.ip = self.get_parameter('robot_ip').get_parameter_value().string_value

    def create_subscribers(self):
        super().create_subscribers()
        self._communication_interface.define_subscribers({
            '/xarm6/gripper_cmd': (self.gripper_callback, GripperCommand)
        })        

    def emergency_stop_callback(self, msg):
        """
            This function is called when an emergency stop is requested.
            It stops the robot if the message is True.
        """
        self.get_logger().info(f"Emergency stop: {msg}")
        if self.simulation_only: # No need to stop the simulation
            return
        if msg:
            self.arm.emergency_stop()
        else:
            self.arm.set_state(state=0)

    def joints_callback(self, cmd):
        """
            This function is called when a new joint value is received from the Operator PC.
            It computes the robot joint values regarding the initial position of the robot,
            and it moves the robot to the new joint values.
        """
        if not self.initialised:
            return
        self.get_logger().info(f"JOINTS: {cmd}") # temp for test
        try:
            cmd_rel = [a - b for a, b in zip(cmd, self.origin_command_device)]
            #abs_cmd = [a + b for a, b in zip(self.jav_init_pos, cmd_rel)]

            #self.get_logger().info(f"ABS CM: {abs_cmd}") # temp for test
            #self.move_robot(abs_cmd)
            self.move_robot(cmd)

        except AttributeError:
            self.origin_command_device = cmd

        self.get_logger().info("====")

    def _transform_unity_to_robot(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the Unity coordinate system to the robot coordinate system.
            The transformation is done by multiplying the position and rotation by the transformation matrix.
        """
        cmd = np.array(cmd)
        trans = np.dot(self.transformation_matrix, cmd[:3]) # Apply the transformation matrix to have the good coordinate for the end effector position
        rot = np.dot(self.transformation_matrix, cmd[3:]) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, rot])
        return cmd
    
    def _transform_robot_to_unity(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the robot coordinate system to the Unity coordinate system.
            The transformation is done by multiplying the position and rotation by the inverse of the transformation matrix.
        """
        cmd = np.array(cmd)
        trans = np.dot(self.inv_transformation_matrix, cmd[:3]) # Apply the transformation matrix to have the good coordinate for the end effector position
        rot = np.dot(self.inv_transformation_matrix, cmd[3:]) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, rot])
        return cmd

    def end_effector_position_callback(self, cmd):
        """
            This function is called when a new end effector position is received from the Operator PC.
            It calls the move_robot function.
        """
        if not self.initialised:
            return
        
        self.ee_cmd = cmd # cmd = self._transform_unity_to_robot(cmd)
        self.get_logger().info(f"End-effector command received: {self.ee_cmd}")
        

    def move_robot(self, pos_or_joint_values, position=False, arm_leg=ct.ArmLeg.ARM, limb_side=ct.ArmLegSide.LEFT, wait=False):
        """
            Move the robot to the given joint values or end effector position.
            The joint values are received as a list of 6 float values, representing the joint angles in degree.
            The end effector position is received as a list of 3 float values, representing the position in mm.

            :param pos_or_joint_values: (List[float]) the joint values or end effector position to move the robot to.
            :param position: (Bool) if True, the pos_or_joint_values is an end effector position. If False, it is joint values.
            :param arm_leg: (ct.ArmLeg) the arm or leg side to move. Default is ARM.
            :param limb_side: (ct.ArmLegSide) the arm id to move. Default is LEFT.
            :param wait: (Bool) if True, the function waits until the robot reaches the desired position.

            TODO: The speed and acceleration values are to be changed.
        """
        if not self.initialised:
            return
        if self.simulation_only:
            self.simulation(pos_or_joint_values, position, arm_leg, limb_side, wait)
            return
        
        if position: # end effector position
            if pos_or_joint_values is None and self.ee_cmd is not None:

                target = np.array(self.ee_cmd, dtype=float)
                _, pose = self.arm.get_position(is_radian=False)
                current = np.array(pose, dtype=float)

                # Position interpolation:
                delta_pose = target[:3] - current[:3]
                distance = np.linalg.norm(delta_pose)
                if distance < 1:
                    cmd_xyz = current[:3]
                else:
                    step_size = min(5.0, distance)
                    cmd_xyz = current[:3] + (delta_pose / distance)

                # Interpolation de l'orientation aussi
                delta_orientation = target[3:] - current[3:]
                orientation_distance = np.linalg.norm(delta_orientation)

                if orientation_distance < 1.0:  # 1 degré
                    cmd_orientation = target[3:]
                else:
                    # Pas d'interpolation pour l'orientation (2 degrés par cycle)
                    step_size_orientation = min(2.0, orientation_distance)
                    cmd_orientation = current[3:] + (delta_orientation / orientation_distance) * step_size_orientation

                command = np.concatenate((cmd_xyz, target[3:]))
                self.get_logger().info(f"target: {target}")
                self.get_logger().info(f"position: {pose}")
                self.ee_cmd = None
            elif self.ee_cmd is None:
                return
            else:
                command = pos_or_joint_values

            self.arm.set_servo_cartesian(
                command,
                speed=self.TRANS_SPEED,
                mvacc=50.0,
            )            
        else: # joint values
            self.get_logger().info(f"Moving joint of robot to: {pos_or_joint_values}")
            try:
                self.arm.set_state(4)
                if self.arm.mode != 1:                        
                    self.arm.set_mode(1)
                self.arm.set_state(state=0)
                ret = self.arm.set_servo_angle(angle=pos_or_joint_values, speed=self.ROT_SPEED, is_radian=False, wait=wait)
                time.sleep(0.1)
                self.get_logger().info(f"Moved, Robot joint values: {self.arm.get_servo_angle()[1][:self.ARM_JOINTS_NUMBER]} (code = {ret})")
            except Exception as e:
                self.get_logger().error("Impossible to move the robot: " + e)

            self._communication_interface.publish("robot_joint_values", pos_or_joint_values)

        self.jav_current_pos[ct.ArmLeg.ARM] = self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]
        self._communication_interface.publish("robot_joint_values", self._transform_robot_to_unity(self.jav_current_pos.value).tolist())

    def simulation(cmd, position=False, arm_leg=ct.ArmLeg.ARM, limb_side=ct.ArmLegSide.LEFT, wait=False):
        pass

    def set_init_position_to_current(self):
        temp = self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]
        self.ee_cmd = temp
        self.jav_init_pos[ct.ArmLeg.ARM] = temp

        self.eec_current_pos = ct.EndEffectorCoordinates(self.arm.get_position(is_radian=False)[1][:6] if self.arm.get_position()[0] == 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # if the get_position() returns an error (not 0 for code), the position is set to [0, 0, 0]
        self.get_logger().info(f"EE Init position: {self.eec_current_pos}")

    def hand_open_close_callback(self, msg):
        arm_id = int(msg.data[0])
        gripper_state = float(msg.data[1])
        self.get_logger().info(f"Arm ID: {arm_id}, Gripper state: {gripper_state}")

        if not self.simulation_only:
            if arm_id == int(ct.ArmLegSide.LEFT):
                self.arm.set_gripper_position() # TODO: implement the gripper position ; def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):
            else:
                self.get_logger().error(f"Unknown arm ID: {arm_id}")

    def gripper_callback(self, msg):
        pos_mm = np.clip(msg.position, -10.0, 850.0)  # -10 to 85 mm for xArm gripper
        speed = np.clip(msg.max_effort, 350.0, 5000.0)

        #self.arm.set_gripper_position(pos_mm, speed=speed, wait=False)
        
        # self.get_logger().info(
        #     f"Gripper command received: {(pos_mm/10.):.2f} mm, "
        #     f"speed: {speed:.2f} r/min"
        # )

    def _anti_lock_timer_callback(self) -> None:
        """
        Periodically re-enable motion and reset state to avoid controller lock.
        """
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)

def main(args=None):
    rclpy.init(args=args)

    controller = XarmController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
