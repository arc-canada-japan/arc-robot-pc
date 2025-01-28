from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from robot_control.abstract_controller import AbstractController
import robot_control.controller_tools as ct
from geometry_msgs.msg import PoseArray, Pose

import time

class KinovaController(AbstractController):
    """
        This class is used to controller the Kinova robot.
        It receives the joint values from the Operator PC and moves the robot to the new joint values.
        The values are received as a list of 6 float values, representing the joint angles in degree.
    """

    def __init__(self):
        super().__init__(robot_name="kinova")

        self.declare_ros_parameters()

        # Setting the defaut values for the robot (since kinova is only one arm, the default values are for the left arm) to access the value simply with ".value"
        self.jav_home_pos.default_limb      = ct.ArmLeg.ARM
        self.jav_home_pos.default_side      = ct.ArmLegSide.LEFT
        self.jav_init_pos.default_limb      = ct.ArmLeg.ARM
        self.jav_init_pos.default_side      = ct.ArmLegSide.LEFT
        self.jav_current_pos.default_limb   = ct.ArmLeg.ARM
        self.jav_current_pos.default_side   = ct.ArmLegSide.LEFT

        self.transformation_matrix = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]]) # Transformation matrix from Unity to robot coordinate system
        self.inv_transformation_matrix = np.linalg.inv(self.transformation_matrix) # Inverse transformation matrix from Unity to robot coordinate system

        if not self.simulation_only:
            pass
        else:
            self.jav_init_pos.value = self.jav_home_pos.value
            self.jav_current_pos.value = self.jav_home_pos.value
            self.eec_current_pos = ct.EndEffectorCoordinates()

        self._communication_interface.define_publishers({
            'vr_pose': PoseArray
        })

        self.get_logger().info(f"init_pos: {self.jav_init_pos}")
        self.get_logger().info(f"home_pos: {self.jav_home_pos}")

        self._init_done()

    def __del__(self):
        if not self.simulation_only and self.arm is not None:
            pass

    def declare_ros_parameters(self):
        """
            Declare the ROS parameters used by the controller.
        """
        pass

    def emergency_stop_callback(self, msg):
        """
            This function is called when an emergency stop is requested.
            It stops the robot if the message is True.
        """
        self.get_logger().info(f"Emergency stop: {msg}")
        if self.simulation_only: # No need to stop the simulation
            return
        if msg:
            pass # TODO: stop
        else:
            pass # TODO: release

    def joints_callback(self, cmd):
        """
            This function is called when a new joint value is received from the Operator PC.
            It computes the robot joint values regarding the initial position of the robot,
            and it moves the robot to the new joint values.
        """
        self.get_logger().warning("joints_callback not implemented for Kinova robot")

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
        
        controller_cmd = cmd.data[:7] # Get the end effector position and quaternion from the message
        head_cmd = cmd.data[7:] # Get the head position and quaternion from the message

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "world"

        head_pos_msg = Pose()
        head_pos_msg.position.x = head_cmd[0]
        head_pos_msg.position.y = head_cmd[1]
        head_pos_msg.position.z = head_cmd[2]
        head_pos_msg.orientation.x = head_cmd[3]
        head_pos_msg.orientation.y = head_cmd[4]
        head_pos_msg.orientation.z = head_cmd[5]
        head_pos_msg.orientation.w = head_cmd[6]

        hand_pos_msg = Pose()
        hand_pos_msg.position.x = controller_cmd[0]
        hand_pos_msg.position.y = controller_cmd[1]
        hand_pos_msg.position.z = controller_cmd[2]
        hand_pos_msg.orientation.x = controller_cmd[3]
        hand_pos_msg.orientation.y = controller_cmd[4]
        hand_pos_msg.orientation.z = controller_cmd[5]
        hand_pos_msg.orientation.w = controller_cmd[6]

        pose_array_msg.poses = [head_pos_msg, hand_pos_msg]

        self.move_robot(pose_array_msg, position=True)

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
            self.get_logger().info(f"Moving EE of robot to: {pos_or_joint_values}")
            if not self.simulation_only:
                self._communication_interface.publish("vr_pose", pos_or_joint_values)
            else:
                pass # TODO: implement the simulation
        else: # joint values
            self.get_logger().warning("joints values not implemented for Kinova robot")

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
                pass #self.arm.set_gripper_position() # TODO: implement the gripper position ; def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):
            else:
                self.get_logger().error(f"Unknown arm ID: {arm_id}")

def main(args=None):
    rclpy.init(args=args)

    controller = KinovaController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()