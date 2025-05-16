from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from robot_control.abstract_controller import AbstractController
import robot_control.controller_tools as ct
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import SetBool

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
        self.define_services()

        self.transformation_matrix = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]]) # Transformation matrix from Unity to robot coordinate system
        self.inv_transformation_matrix = np.linalg.inv(self.transformation_matrix) # Inverse transformation matrix from Unity to robot coordinate system

        self.kinova_tracking_enabled = False

        self._init_done() # last function to call in the constructor

    def __del__(self):
        if not self.simulation_only:
            pass

    def declare_ros_parameters(self):
        """
            Declare the ROS parameters used by the controller.
        """
        pass

    def create_publishers(self):
        """
            Create the publishers used by the controller.
        """
        super().create_publishers()

        self._communication_interface.define_publishers({
            'vr_pose': PoseArray
        })

    def define_services(self):
        """
            Define the services used by the controller.
        """
        self.client = self.create_client(SetBool, 'set_gripper_state')        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        self.client_tracking = self.create_client(SetBool, 'toggle_tracking')
        while not self.client_tracking.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for tracking service...')


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
    
    def _transform_robot_to_unity(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the robot coordinate system to the Unity coordinate system.
            The transformation is done by multiplying the position and rotation by the inverse of the transformation matrix.
        
            TODO: update with quaternion
        """
        cmd = np.array(cmd)
        trans = np.dot(self.inv_transformation_matrix, cmd[:3]) # Apply the transformation matrix to have the good coordinate for the end effector position
        rot = np.dot(self.inv_transformation_matrix, cmd[3:]) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, rot])
        return cmd

    def _transform_unity_to_robot(self, cmd):
        """
            This function is used to transform the end effector position and rotation from the Unity coordinate system to the robot coordinate system.
            The transformation is done by multiplying the position and rotation by the transformation matrix.
        """
        cmd = np.array(cmd)
        trans = np.dot(self.transformation_matrix, cmd[:3]) # Apply the transformation matrix to have the good coordinate for the end effector position
        quat = self._apply_transformation_to_quaternion(cmd[3:], self.transformation_matrix) # Apply the transformation matrix to have the good coordinate for the end effector rotation
        cmd = np.concatenate([trans, quat])
        return cmd

    def _apply_transformation_to_quaternion(self, quaternion, transformation_matrix):
        """
        Apply a transformation matrix (rotation) to a quaternion.

        Args:
            quaternion (list or np.ndarray): Quaternion [w, x, y, z] to transform.
            transformation_matrix (np.ndarray): 3x3 rotation matrix representing the transformation.

        Returns:
            np.ndarray: Transformed quaternion [w, x, y, z].
        """
        # Convert quaternion to a scipy Rotation object
        rot = R.from_quat(quaternion)  # scipy expects [x, y, z, w]
        
        # Get the rotation matrix from the quaternion
        R_q = rot.as_matrix()  # Converts quaternion to a 3x3 rotation matrix
        
        # Apply the transformation matrix
        R_new = transformation_matrix @ R_q  # Matrix multiplication

        # Convert the resulting rotation matrix back to a quaternion
        new_quaternion = R.from_matrix(R_new).as_quat()  # Returns [x, y, z, w]
        
        # Reorder to [w, x, y, z] for consistency
        #new_quaternion = np.roll(new_quaternion, shift=1)
        
        return new_quaternion

    def end_effector_position_callback(self, cmd):
        """
            This function is called when a new end effector position is received from the Operator PC.
            It moves the robot to the new end effector position.
            Before moving the robot, it checks if the distance between the current position and the new position is greater than the threshold.
            If the distance is greater, the robot moves to the new position, after applying a low pass filter to the command.
        """
        if not self.initialised:
            return

        #controller_cmd = self._transform_unity_to_robot(cmd[:7]) # Get the end effector position and quaternion from the message
        #head_cmd = self._transform_unity_to_robot(cmd[7:]) # Get the head position and quaternion from the message
        eec_controller = ct.EndEffectorCoordinates(cmd[:7])
        eec_head = ct.EndEffectorCoordinates(cmd[7:])
        # controller_cmd = cmd[:7]
        # head_cmd = cmd[7:]

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "world"

        # head_pos_msg = Pose()
        # head_pos_msg.position.x = head_cmd[0]
        # head_pos_msg.position.y = head_cmd[1]
        # head_pos_msg.position.z = head_cmd[2]
        # head_pos_msg.orientation.x = head_cmd[3]
        # head_pos_msg.orientation.y = head_cmd[4]
        # head_pos_msg.orientation.z = head_cmd[5]
        # head_pos_msg.orientation.w = head_cmd[6]

        # hand_pos_msg = Pose()
        # hand_pos_msg.position.x = controller_cmd[0]
        # hand_pos_msg.position.y = controller_cmd[1]
        # hand_pos_msg.position.z = controller_cmd[2]
        # hand_pos_msg.orientation.x = controller_cmd[3]
        # hand_pos_msg.orientation.y = controller_cmd[4]
        # hand_pos_msg.orientation.z = controller_cmd[5]
        # hand_pos_msg.orientation.w = controller_cmd[6]

        # pose_array_msg.poses = [head_pos_msg, hand_pos_msg]
        pose_array_msg.poses = [eec_head.to_pose_msg(), eec_controller.to_pose_msg()]

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
            #self.get_logger().info(f"Moving EE of robot to: {pos_or_joint_values}")
            if not self.simulation_only:
                self._communication_interface.publish("vr_pose", pos_or_joint_values)
            else:
                pass # TODO: implement the simulation
        else: # joint values
            self.get_logger().warning("joints values not implemented for Kinova robot")       


    def set_init_position_to_current(self):
        self.get_logger().warning("set_init_position_to_current not implemented for Kinova robot")
        return
        temp = self.arm.get_servo_angle(is_radian=False)[1][:self.ARM_JOINTS_NUMBER]
        self.jav_init_pos[ct.ArmLeg.ARM] = temp

        self.eec_current_pos = ct.EndEffectorCoordinates(self.arm.get_position(is_radian=False)[1][:6] if self.arm.get_position()[0] == 0 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # if the get_position() returns an error (not 0 for code), the position is set to [0, 0, 0]
        self.get_logger().info(f"EE Init position: {self.eec_current_pos}")

    def hand_open_close_callback(self, msg):
        arm_id = int(msg[0])
        gripper_state = float(msg[1])
        self.get_logger().info(f"Arm ID: {arm_id}, Gripper state: {gripper_state}")
        gripper_bool = True if int(gripper_state) == 1 else False

        if not self.simulation_only:
            if arm_id == int(ct.ArmLegSide.LEFT.value) or arm_id == int(ct.ArmLegSide.RIGHT.value):
                self.send_request(gripper_bool)
            else:
                self.get_logger().error(f"Unknown arm ID: {arm_id}")

    def controller_activated_callback(self, msg):
        arm_id = int(msg[0])
        controller_state = (float(msg[1]) == 1.0)

        if not self.simulation_only:
            if arm_id == int(ct.ArmLegSide.LEFT.value) or arm_id == int(ct.ArmLegSide.RIGHT.value):
                if self.kinova_tracking_enabled != controller_state: 
                    self.send_request_tracking(True)
                    self.kinova_tracking_enabled = controller_state
                    self.get_logger().info(f"Arm ID: {arm_id}, controller state: {controller_state}")
            else:
                self.get_logger().error(f"Unknown arm ID: {arm_id}")

    def send_request(self, state: bool):
        req = SetBool.Request()
        req.data = state  # True to close, False to open

        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def send_request_tracking(self, state: bool):
        req = SetBool.Request()
        req.data = state  # True to close, False to open

        future = self.client_tracking.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: Success={response.success}, Message={response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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