from typing import List
import rclpy
from rclpy.node import Node
from robot_control.abstract_controller import AbstractController, ArmLeg, ArmLegSide, ArticulationDesc

from std_msgs.msg import String, Bool, Float32MultiArray
from xarm.wrapper import XArmAPI

RPM_TO_RAD_S = 0.10472 # mutiply with a rpm value to get the rad/s value
RPM_TO_DEG_S = 6 # mutiply with a rpm value to get the °/s value

class XarmController(AbstractController):
    """
        This class is used to controller the xArm robot.
        It receives the joint values from the Operator PC and moves the robot to the new joint values.
        The values are received as a list of 6 float values, representing the joint angles in degree.
    """

    def __init__(self):
        super().__init__(robot_name="xarm")

        self.declare_ros_parameters()

        self.SPEED = 50 # r/min
        self.SPEED = self.SPEED * RPM_TO_DEG_S # conversion to °/s 

        if not self.simulation_only:
            try:
                self.arm = XArmAPI(self.ip, is_radian=False)
            except Exception as e:
                self.get_logger().error("Impossible to connect to the robot: " + e)
                exit(10)

            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(state=0)
            self.arm.clean_error()
            self.arm.clean_warn()
            #self.arm.reset(wait=True)

            self.set_init_position_to_current()
        else:
            self.init_pos = self.home_pos

        self.get_logger().info(f"init_pos: {self.init_pos}")
        self._communication_interface.publish("robot_joint_values",
                                              self.init_pos[ArmLeg.ARM.value][ArmLegSide.LEFT.value]
                                            )

        self.get_logger().info("========= "+self.ROBOT_NAME+" CONTROLLER INIT DONE =========")   

    def declare_ros_parameters(self):
        """
            Declare the ROS parameters used by the controller.
        """
        # Get the robot IP address from the parameter send by the launch file
        self.declare_parameter('robot_ip', '192.168.1.217')
        self.ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        #self.declare_parameter('robot_home_position', self.home_pos)
        #self.home_pos = self.get_parameter('robot_home_position').get_parameter_value()._double_array_value

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
        self.get_logger().info(f"JOINTS: {cmd}") # temp for test
        try:
            cmd_rel = [a - b for a, b in zip(cmd, self.origin_command_device)]
            abs_cmd = [a + b for a, b in zip(self.init_pos, cmd_rel)]

            self.get_logger().info(f"ABS CM: {abs_cmd}") # temp for test
            self.move_robot(abs_cmd)

        except AttributeError:
            self.origin_command_device = cmd

        self.get_logger().info("====")

    def move_robot(self, joint_values, arm_leg=ArmLeg.ARM, limb_side=ArmLegSide.LEFT):
        """
            Move the robot to the new joint values.
            The joint values are received as a list of 6 float values, representing the joint angles in degree.
        """
        self.get_logger().info(f"Moving robot to: {joint_values}")
        if not self.simulation_only:
            self.arm.set_servo_angle(angle=joint_values, speed=self.SPEED, is_radian=False, wait=True)

        self._communication_interface.publish("robot_joint_values", joint_values)

    def set_init_position_to_current(self):
        init_pos = {
            ArmLegSide.LEFT.value: None,
            ArmLegSide.RIGHT.value: None
        }
        temp = ArticulationDesc()
        temp = self.arm.get_servo_angle()[1][:self.JOINTS_NUMBER]
        init_pos[ArmLegSide.LEFT.value] = temp
        self.init_pos[ArmLeg.ARM] = init_pos

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