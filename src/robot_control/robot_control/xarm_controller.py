import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray
from xarm.wrapper import XArmAPI

import matplotlib.pyplot as plt # temp for test
import matplotlib.animation as animation # temp for test

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

        self.SPEED = 50 # tr/min
        self.SPEED = self.SPEED * 0.10472 # conversion to rad/s 

        self.arm = XArmAPI(ip, is_radian=True)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        #self.arm.reset(wait=True)

        self.arm_origin = self.arm.get_servo_angle()[1][:6]
        self.get_logger().info(f"ORIGIN: {self.arm_origin}")

        self.graph_init()

    def graph_init(self):
        self.history = {'cmd_data': [], 'cmd_rel': [], 'abs_cmd': []}
        # Set up the figure and axis for real-time plotting
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.lines = {}
        joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.time_steps = list(range(1, len(joints) + 1))

        self.lines['cmd_data'], = self.ax.plot([], [], label='cmd.data', marker='o')
        self.lines['cmd_rel'], = self.ax.plot([], [], label='cmd_rel', marker='x')
        self.lines['abs_cmd'], = self.ax.plot([], [], label='abs_cmd', marker='s')

        self.ax.set_xlabel('Joint')
        self.ax.set_ylabel('Angle (radian)')
        self.ax.set_xticks(self.time_steps)
        self.ax.set_xticklabels(joints)
        self.ax.set_title(f'Joint Angles over Time\narm_origin: {self.arm_origin}\norigin: xxxx')
        self.ax.legend()
        self.ax.grid(True)

        plt.ion()
        plt.show()
        self.get_logger().info(f"SHOW")


    def joint_print_callback(self, msg):
        #self.get_logger().info('"%s"' % msg.data)
        pass

    def emergency_stop_callback(self, msg):
        self.get_logger().info(f"Emergency stop: {msg.data}")
        if msg.data:
            self.arm.emergency_stop()
        else:
            self.arm.set_state(state=0)

    def robot_move_callback(self, cmd):
        self.get_logger().info(f"JOINTS: {cmd.data}") # temp for test
        try:
            cmd_rel = Float32MultiArray()
            cmd_rel = [a - b for a, b in zip(cmd.data, self.origin)]
            abs_cmd = Float32MultiArray()
            abs_cmd = [a + b for a, b in zip(self.arm_origin, cmd_rel)]

            self.get_logger().info(f"ABS CM: {abs_cmd}") # temp for test
            #self.arm.set_servo_angle(angle=abs_cmd, speed=self.SPEED, is_radian=True, wait=True)

            # Add data to history
            self.history['cmd_data'].append(cmd.data)
            self.history['cmd_rel'].append(cmd_rel)
            self.history['abs_cmd'].append(abs_cmd)

            if len(self.history['cmd_data']) > 50:  # Keep history to the last 10 entries
                self.history['cmd_data'].pop(0)
                self.history['cmd_rel'].pop(0)
                self.history['abs_cmd'].pop(0)

            # Update the plot
            self.update_plot()

        except AttributeError:
            self.origin = cmd.data
            # Format the float numbers to two decimal places
            arm_origin_str = ', '.join(f'{num:.2f}' for num in self.arm_origin)
            origin_str = ', '.join(f'{num:.2f}' for num in self.origin)
            self.ax.set_title(f'Joint Angles over Time\narm_origin: [{arm_origin_str}]\norigin: [{origin_str}]')

        self.get_logger().info("====")

    def update_plot(self):
        self.lines['cmd_data'].set_data(self.time_steps, self.history['cmd_data'][-1])
        self.lines['cmd_rel'].set_data(self.time_steps, self.history['cmd_rel'][-1])
        self.lines['abs_cmd'].set_data(self.time_steps, self.history['abs_cmd'][-1])

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

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