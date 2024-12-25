from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from robot_control.abstract_controller import AbstractController
import robot_control.controller_tools as ct

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

        self.TRANS_SPEED = 100 # mm/s
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

        self.eec_controller_pos_time = ct.DataBuffer(10) # Buffer to store the controller end effector position for the low pass filter
        self.eec_controller_last_pos = None

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
            #self.arm.reset(wait=False)

            self.arm.set_mode(1)
            self.arm.set_state(state=0)
            #self.arm.move_gohome(wait=True)
            #self.arm.reset(wait=False)
            time.sleep(1)
            self.set_init_position_to_current()
            self.move_robot(self.jav_home_pos.value, wait=True)
            self.arm.set_mode(7)
            self.arm.set_state(state=0)
            time.sleep(1)
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
            #self.arm.set_mode(0)
            #self.arm.set_state(0)
            #self.arm.move_gohome(wait=True)
            self.arm.disconnect()

    def declare_ros_parameters(self):
        """
            Declare the ROS parameters used by the controller.
        """
        # Get the robot IP address from the parameter send by the launch file
        self.declare_parameter('robot_ip', '192.168.1.217')
        self.ip = self.get_parameter('robot_ip').get_parameter_value().string_value

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
            It moves the robot to the new end effector position.
            Before moving the robot, it checks if the distance between the current position and the new position is greater than the threshold.
            If the distance is greater, the robot moves to the new position, after applying a low pass filter to the command.
        """
        if not self.initialised:
            return
        # Ensure that cmd and self.eec_current_pos are numpy arrays
        cmd = self._transform_unity_to_robot(cmd) * self.MOVE_FACTOR # Apply the transformation matrix to have the good coordinate, and the move factor
        #self.eec_current_pos = np.array(self.eec_current_pos)
        self.eec_current_pos.coordinates = self.arm.get_position(is_radian=False)[1][:6]
        
        if self.eec_controller_last_pos is None:
            self.eec_controller_last_pos = ct.EndEffectorCoordinates(cmd)
            return

        if not self.eec_controller_pos_time.is_full():
            self.eec_controller_pos_time.append(cmd)
            return

        distance = np.linalg.norm(cmd[:3] - self.eec_controller_last_pos.position) # Compute the distance between the current position and the new position
        self.get_logger().info(f"0. Current Robot EE position: {self.eec_current_pos}")
        self.get_logger().info(f"0.5. Current controller position: {cmd}")
        self.get_logger().info(f"1. Distance: {distance}")
        
        # If the distance exceeds the threshold, move the robot
        if distance > (self.END_EFFECTOR_EPSILON * self.MOVE_FACTOR):
            #self.get_logger().info(f"Cmd: {cmd}")
            #self.get_logger().info(f"Current pos: {self.eec_current_pos}")
            #self.get_logger().info(f"filtered: {self.eec_controller_pos_time.lowpass_filter(0.1)}")
            new_position = self.eec_current_pos.coordinates + (self.eec_controller_pos_time.mean() - self.eec_controller_last_pos.coordinates) # TODO: filtering on the delta position
            self.get_logger().info(f"2. Mean: {self.eec_controller_pos_time.mean()}")
            self.get_logger().info(f"3. Last controller position: {self.eec_controller_last_pos}")
            self.get_logger().info(f"4. Delta: {(self.eec_controller_pos_time.mean() - self.eec_controller_last_pos.coordinates)}")
            self.get_logger().info(f"5. New EE robot position: {new_position}")
            self.move_robot(new_position.tolist(), position=True)
            # Update the controller position to the latest command
            #self.eec_controller_pos_time.append(cmd)
            self.eec_controller_pos_time.clear()
            self.eec_controller_last_pos.coordinates = cmd
            self.get_logger().info(f"8. New last controller position: {self.eec_controller_last_pos.coordinates}")

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
        if position: # end effector position
            self.get_logger().info(f"6. Moving EE of robot to: {pos_or_joint_values}")
            if not self.simulation_only:
                # self.arm.set_state(4)
                # time.sleep(0.01)
                # if self.arm.mode != 7:
                #     self.arm.set_mode(7)
                #     self.arm.set_state(state=0)
                                    
                ret = self.arm.set_position(pos_or_joint_values[0], pos_or_joint_values[1], pos_or_joint_values[2],
                                      #pos_or_joint_values[3], pos_or_joint_values[4], pos_or_joint_values[5], 
                                      speed=self.TRANS_SPEED, wait=wait, is_radian=False)
                # ret = self.arm.set_servo_cartesian(pos_or_joint_values, speed=self.TRANS_SPEED, is_radian=False)
                self.get_logger().info(f"6.5 ret: {ret}")
                time.sleep(0.1)
                #self._communication_interface.publish("robot_joint_values", self._transform_robot_to_unity(self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]).tolist())
                #self.eec_current_pos = pos_or_joint_values#[:3]
                #checked_pos = self.arm.get_position(is_radian=False)
                #self.eec_current_pos = checked_pos[1][:6] if checked_pos[0] == 0 else pos_or_joint_values # if the get_position() returns an error (not 0 for code), the position is set to [0, 0, 0]
                #self.get_logger().info(f"7. Moved, Robot EE position: {self.eec_current_pos} (code = {checked_pos[0]})")
            else:
                pass # TODO: implement the simulation
        else: # joint values
            self.get_logger().info(f"Moving joint of robot to: {pos_or_joint_values}")
            if not self.simulation_only:
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

            #self._communication_interface.publish("robot_joint_values", pos_or_joint_values)

        self.jav_current_pos[ct.ArmLeg.ARM] = self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]
        self._communication_interface.publish("robot_joint_values", self._transform_robot_to_unity(self.jav_current_pos.value).tolist())


    def set_init_position_to_current(self):
        temp = self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]
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