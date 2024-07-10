import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray
from xarm.wrapper import XArmAPI

class XarmController(Node):

    def __init__(self):
        super().__init__('xarm_controller')
        self.get_logger().info("========= XARM CONTROLLER =========")

        self.declare_parameter('robot_ip', '192.168.1.217')
        ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            String,
            'joint_value_str',
            self.joint_print_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.emergency = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10)
        self.emergency  # prevent unused variable warning

        self.joints_val = self.create_subscription(
            Float32MultiArray,
            'joint_value',
            self.robot_move_callback,
            10)
        self.joints_val  # prevent unused variable warning

        self.SPEED = 50

        self.arm = XArmAPI(ip, is_radian=True)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

    def joint_print_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def emergency_stop_callback(self, msg):
        self.get_logger().info(f"Emergency stop: {msg.data}")
        if msg.data:
            self.arm.emergency_stop()
        else:
            self.arm.set_state(state=0)

    def robot_move_callback(self, cmd):
        self.get_logger().info(f"JOINTS: {cmd.data}")

        return
        self.arm.set_servo_angle(servo_id=1, angle=90, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))
        self.arm.set_servo_angle(servo_id=3, angle=-60, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))
        self.arm.set_servo_angle(servo_id=2, angle=-30, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))
        self.arm.set_servo_angle(servo_id=1, angle=0, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))
        self.arm.set_servo_angle(servo_id=2, angle=0, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))
        self.arm.set_servo_angle(servo_id=3, angle=0, speed=self.SPEED, is_radian=False, wait=True)
        print(self.arm.get_servo_angle(), self.arm.get_servo_angle(is_radian=False))



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